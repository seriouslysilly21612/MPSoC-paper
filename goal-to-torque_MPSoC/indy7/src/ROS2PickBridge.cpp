/*****************************************************************************
*	Name: ROS2PickBridge.cpp
*	Description: The ROS 2 goal bridge (paper, Sec. II and V-C); see
*	ROS2PickBridge.h for the threads and what the real-time tasks may call.
*****************************************************************************/
#include "ROS2PickBridge.h"
#include "obs/ObsMethod.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <pthread.h>

using namespace std::chrono_literals;

static const char* TOPIC_TARGET     = "/pick_target_base";
static const char* TOPIC_DETECTIONS = "/detections";
static const char* TOPIC_JOINTS     = "/joint_states";
static const char* TOPIC_STATUS     = "/contract_status";
static const char* PICK_LOGIC_NODE  = "/pick_logic_node";
static const char* PARAM_DESIRED    = "desired_class";

CROS2PickBridge::CROS2PickBridge()
{
}

CROS2PickBridge::~CROS2PickBridge()
{
	DeInit();
}

double
CROS2PickBridge::NowS()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

BOOL
CROS2PickBridge::Init(const Config& astCfg, CGoalSlot* apSlot)
{
	if (m_bInit)
		return TRUE;
	if (apSlot == NULL)
		return FALSE;
	m_stCfg = astCfg;
	m_pSlot = apSlot;
	m_cFilter.SetParams(astCfg.stFilter);
	try
	{
		if (!rclcpp::ok())
		{
			// the controller handles SIGINT and SIGTERM itself (main.cpp):
			// rclcpp must not install its own handlers
			rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
			m_bOwnContext = TRUE;
		}
		m_pNode = std::make_shared<rclcpp::Node>("indy7_bridge");
		m_pSubTarget = m_pNode->create_subscription<my_interfaces::msg::PickTarget3D>(
			TOPIC_TARGET, rclcpp::QoS(10),
			[this](const my_interfaces::msg::PickTarget3D::SharedPtr apMsg) { OnTarget(apMsg); });
		m_pSubDetections = m_pNode->create_subscription<my_interfaces::msg::DetectionArray>(
			TOPIC_DETECTIONS, rclcpp::QoS(10),
			[this](const my_interfaces::msg::DetectionArray::SharedPtr apMsg) { OnDetections(apMsg); });
		m_pPubJoints = m_pNode->create_publisher<sensor_msgs::msg::JointState>(TOPIC_JOINTS, rclcpp::QoS(10));
		if (astCfg.pStatus != NULL)
		{
			// on the executor: the worker waits for services (seconds), and the detector does not trust an old status
			m_pPubStatus = m_pNode->create_publisher<my_interfaces::msg::ContractStatus>(TOPIC_STATUS, rclcpp::QoS(1));
			m_pStatusTimer = m_pNode->create_wall_timer(20ms, [this]() { PublishStatus(); });
		}
		m_pParamClient = std::make_shared<rclcpp::AsyncParametersClient>(m_pNode, PICK_LOGIC_NODE);
		m_pExec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		m_pExec->add_node(m_pNode);
		m_bStop = false;
		m_thSpin = std::thread(&CROS2PickBridge::SpinLoop, this);
		m_thWorker = std::thread(&CROS2PickBridge::WorkerLoop, this);
	}
	catch (const std::exception& e)
	{
		printf("[bridge] cannot start: %s (were /opt/ros/humble/setup.bash and ~/ros2_ws/install/setup.bash "
		       "sourced before the controller? run.sh does it)\n", e.what());
		m_bStop = true;
		if (m_pExec)
			m_pExec->cancel();
		if (m_thSpin.joinable())
			m_thSpin.join();
		if (m_thWorker.joinable())
			m_thWorker.join();
		m_pExec.reset();
		m_pParamClient.reset();
		m_pStatusTimer.reset();
		m_pPubStatus.reset();
		m_pPubJoints.reset();
		m_pSubDetections.reset();
		m_pSubTarget.reset();
		m_pNode.reset();
		if (m_bOwnContext && rclcpp::ok())
			rclcpp::shutdown();
		m_bOwnContext = FALSE;
		return FALSE;
	}
	m_bInit = TRUE;
	const CGoalFilter::Params& p = astCfg.stFilter;
	if (astCfg.pObs != NULL)
		printf("[bridge] up: %s -> mailbox (the control task checks and averages; %s), %s at 20 Hz%s%s\n",
		       TOPIC_TARGET, astCfg.bTagFormat ? "the object from the tool tag of its image" : "the object in base_link",
		       TOPIC_JOINTS, astCfg.pStatus != NULL ? ", the status of the control task on " : "",
		       astCfg.pStatus != NULL ? TOPIC_STATUS : "");
	else
		printf("[bridge] up: %s -> goal slot (box x [%.2f, %.2f] y [%.2f, %.2f] z [%.2f, %.2f] m, r <= %.2f m, "
		       "floor %.2f m, hover %.2f m; kappa %.2f, restart after %.1f s), %s at 20 Hz\n",
		       TOPIC_TARGET, p.adBox[0], p.adBox[1], p.adBox[2], p.adBox[3], p.adBox[4], p.adBox[5], p.dRMax, p.dFloor,
		       p.dHover, p.dKappa, p.dGapS, TOPIC_JOINTS);
	printf("[bridge] 'p' the object menu: a digit sets desired_class of %s and the arm follows the object; "
	       "'y', 8 follows the class chosen with 'p'\n", PICK_LOGIC_NODE);
	return TRUE;
}

BOOL
CROS2PickBridge::DeInit()
{
	if (!m_bInit)
		return TRUE;
	m_bInit = FALSE;
	// desired_class is a live parameter of pick_logic and outlives this
	// process: leave the pipeline with no class selected, as the earlier
	// controller did (one short best-effort attempt, while spin() still runs)
	try
	{
		if (m_pParamClient && m_pParamClient->service_is_ready() &&
		    m_pParamClient->set_parameters({rclcpp::Parameter(PARAM_DESIRED, std::string())}).wait_for(500ms) ==
		        std::future_status::ready)
			printf("[bridge] desired_class cleared (session end)\n");
	}
	catch (...) {}
	m_bStop = true;
	if (m_pExec)
		m_pExec->cancel();          // spin() returns
	if (m_thSpin.joinable())
		m_thSpin.join();
	if (m_thWorker.joinable())
		m_thWorker.join();
	if (m_stCfg.pObs != NULL)
	{
		const CObsWriter& cObs = *m_stCfg.pObs;
		printf("[bridge] %ld samples: %ld written into the mailbox; not written: %ld invalid, %ld not finite, "
		       "%ld older than the cap or from the future, %ld with no free slot, %ld past their deadline, %ld without "
		       "a close reading of the counter\n",
		       m_nSamples.load(), cObs.Count(CObsWriter::eWritten), cObs.Count(CObsWriter::eInvalid),
		       cObs.Count(CObsWriter::eNotFinite), cObs.Count(CObsWriter::eAge), cObs.Count(CObsWriter::eFull),
		       cObs.Count(CObsWriter::eExpired), cObs.Count(CObsWriter::eClock));
		printf("[bridge] samples by mode: %ld of 416, %ld of 320%s", m_anMode[0].load(), m_anMode[1].load(),
		       m_stCfg.bTagFormat ? "" : "\n");
		if (m_stCfg.bTagFormat)
			printf("; %ld valid ones without the tool tag in their image (written, the control task refuses them)\n",
			       m_nTagLost.load());
	}
	else
		printf("[bridge] %ld samples: %ld written into the goal slot (%ld restarts of the average); rejected: "
		       "%ld invalid, %ld not finite, %ld below the floor, %ld outside the box\n",
		       m_nSamples.load(), m_nWritten.load(), m_nRestarts.load(),
		       m_anRejected[CGoalFilter::eInvalid].load(), m_anRejected[CGoalFilter::eNotFinite].load(),
		       m_anRejected[CGoalFilter::eBelowFloor].load(), m_anRejected[CGoalFilter::eOutsideBox].load());
	m_pExec.reset();
	m_pParamClient.reset();
	m_pStatusTimer.reset();
	m_pPubStatus.reset();
	m_pPubJoints.reset();
	m_pSubDetections.reset();
	m_pSubTarget.reset();
	m_pNode.reset();
	if (m_bOwnContext && rclcpp::ok())
		rclcpp::shutdown();
	m_bOwnContext = FALSE;
	printf("[bridge] down\n");
	return TRUE;
}

/* ---------------------------------------------------------------- [RT] -- */

BOOL
CROS2PickBridge::FeedDigit(char acDigit)
{
	if (acDigit < '0' || acDigit > '9' || !m_bMenuOpen.load(std::memory_order_acquire))
		return FALSE;
	m_nDigit.store(acDigit - '0', std::memory_order_release);
	return TRUE;
}

void
CROS2PickBridge::PushJointState(const double* adQ, const double* adQd, int anDof)
{
	if (anDof <= 0 || anDof > JS_MAX)
		return;
	JointState stJs;
	memset(&stJs, 0, sizeof(stJs));
	stJs.nDof = anDof;
	for (int i = 0; i < anDof; i++)
	{
		stJs.adQ[i] = adQ[i];
		stJs.adQd[i] = adQd != NULL ? adQd[i] : 0.0;
	}
	m_jointSlot.Write(stJs);
}

/* ------------------------------------------------------------- threads -- */

void
CROS2PickBridge::SpinLoop()
{
	pthread_setname_np(pthread_self(), "bridge-exec");
	m_pExec->spin();                // returns on cancel() or rclcpp::shutdown()
}

void
CROS2PickBridge::WorkerLoop()
{
	pthread_setname_np(pthread_self(), "bridge-work");
	// a class left by an earlier session would make the pipeline publish
	// targets from the start; in pick_logic '' selects nothing. The parameter
	// service is discovered later than the topics (seconds on the KV260).
	try
	{
		bool bReady = false;
		for (int n = 0; n < 10 && !m_bStop.load() && !bReady; n++)
		{
			bReady = m_pParamClient->wait_for_service(500ms);
			if (m_bMenuReq.exchange(false))   // no menu before the reset
				printf("[bridge] waiting for %s to reset desired_class - 'p' again in a moment\n", PICK_LOGIC_NODE);
		}
		if (!bReady)
			printf("[bridge] %s not found: desired_class not reset (start the pipeline before the controller)\n",
			       PICK_LOGIC_NODE);
		else if (SetDesiredClass(std::string()))
			printf("[bridge] desired_class reset for a fresh session: choose the object with 'p'\n");
	}
	catch (const std::exception& e)
	{
		printf("[bridge] desired_class not reset: %s\n", e.what());
	}
	while (!m_bStop.load() && rclcpp::ok())
	{
		std::this_thread::sleep_for(50ms);
		TickJointStates();
		if (m_bMenuReq.exchange(false))
			ShowMenu();
		const int nSel = m_nDigit.exchange(-1);
		if (nSel >= 0)
			HandleDigit(nSel);
		TickReport();
	}
}

/* ------------------------------------------------- executor: the goals -- */

void
CROS2PickBridge::OnTarget(const my_interfaces::msg::PickTarget3D::SharedPtr apMsg)
{
	const double adP[3] = {apMsg->x, apMsg->y, apMsg->z};
	const bool bFrame = apMsg->header.frame_id == "base_link";
	if (!bFrame && !m_bWrongFrame.exchange(true))
		printf("[bridge] /pick_target_base comes in frame '%s', not base_link: its samples are ignored\n",
		       apMsg->header.frame_id.c_str());
	const bool bValid = apMsg->target_valid && apMsg->depth_valid && bFrame;
	const double dNow = NowS();
	if (m_stCfg.pObs != NULL)
	{
		/* the methods with the mailbox: the control task accepts the result, checks the workspace and averages */
		const int64_t tCapture = CObsWriter::MonoOfReal((int64_t)apMsg->capture_stamp.sec * 1000000000LL +
		                                                (int64_t)apMsg->capture_stamp.nanosec);
		const int nMode = apMsg->mode == 1 ? 1 : 0;
		const double adTag[3] = {apMsg->tag_x, apMsg->tag_y, apMsg->tag_z};
		double adZ[3];
		const unsigned uFormat = CObsWriter::FormatOf(m_stCfg.bTagFormat, adP, apMsg->tag_valid, adTag, adZ);
		const CObsWriter::eResult eR = m_stCfg.pObs->Put(bValid, adZ, uFormat, tCapture, nMode, (int)apMsg->class_id);
		m_nSamples.fetch_add(1, std::memory_order_relaxed);
		m_anMode[nMode].fetch_add(1, std::memory_order_relaxed);
		if (bValid && (uFormat & OBS_TAG_LOST))
			m_nTagLost.fetch_add(1, std::memory_order_relaxed);
		if (eR != CObsWriter::eWritten)
			return;
		m_nWritten.fetch_add(1, std::memory_order_relaxed);
		{
			std::lock_guard<std::mutex> cLock(m_Mtx);
			for (int k = 0; k < 3; k++)
				m_adLastGoal[k] = adP[k];
			m_strLastClass = apMsg->class_name;
		}
		m_dLastAcceptS.store(dNow, std::memory_order_release);
		return;
	}
	double adOut[3];
	const CGoalFilter::eVerdict eV = m_cFilter.Push(bValid, adP, dNow, adOut);
	m_nSamples.fetch_add(1, std::memory_order_relaxed);
	if (eV != CGoalFilter::eAccepted)
	{
		m_anRejected[eV].fetch_add(1, std::memory_order_relaxed);
		m_nLastReject.store((int)eV, std::memory_order_relaxed);
		return;
	}
	m_pSlot->Write(adOut);          // the only writer of the slot of the application
	m_nWritten.fetch_add(1, std::memory_order_relaxed);
	if (m_cFilter.Restarted())
		m_nRestarts.fetch_add(1, std::memory_order_relaxed);
	{
		std::lock_guard<std::mutex> cLock(m_Mtx);
		for (int k = 0; k < 3; k++)
			m_adLastGoal[k] = adOut[k];
		m_strLastClass = apMsg->class_name;
	}
	m_dLastAcceptS.store(dNow, std::memory_order_release);
}

/* a time of the clock of the tasks [ns] as a stamp of the system clock, the clock of the image stamps */
static builtin_interfaces::msg::Time
StampOfTaskTime(uint64_t atTaskTime)
{
	const int64_t tSystem = CObsWriter::RealOfMono((int64_t)atTaskTime);
	builtin_interfaces::msg::Time stStamp;
	stStamp.sec = (int32_t)(tSystem / 1000000000LL);
	stStamp.nanosec = (uint32_t)(tSystem % 1000000000LL);
	return stStamp;
}

/* every 20 ms: the newest status of the control task (obs_status.h) for the detector */
void
CROS2PickBridge::PublishStatus()
{
	obs_status_t stStatus;
	uint32_t auStamp[2];
	if (obs_status_read(m_stCfg.pStatus, &stStatus, auStamp) != IPC_OK)
		return;                     // being written, or the control task has not written yet: the next tick
	my_interfaces::msg::ContractStatus stMsg;
	stMsg.stamp = StampOfTaskTime(stStatus.t_release);
	stMsg.tolerance = stStatus.tolerance;
	stMsg.control_period = stStatus.control_period;
	stMsg.response_max = stStatus.response_max;
	stMsg.contract_crc = stStatus.contract_crc;
	stMsg.method = obs_method::Name(stStatus.method);
	stMsg.result_in_use = (stStatus.flags & OBS_STATUS_RESULT_IN_USE) != 0;
	stMsg.target_lost = (stStatus.flags & OBS_STATUS_TARGET_LOST) != 0;
	stMsg.result_n = stStatus.result_n;
	if (stStatus.result_n != 0)     // else no result so far: its times stay 0
	{
		stMsg.result_capture = StampOfTaskTime(stStatus.result_t_e);
		stMsg.result_valid_until = StampOfTaskTime(stStatus.result_t_valid_until);
	}
	stMsg.result_error_bound = stStatus.result_error_bound;
	stMsg.result_mode = stStatus.result_mode;
	stMsg.target_seq = stStatus.target_seq;
	m_pPubStatus->publish(stMsg);
}

void
CROS2PickBridge::OnDetections(const my_interfaces::msg::DetectionArray::SharedPtr apMsg)
{
	const double dNow = NowS();
	std::lock_guard<std::mutex> cLock(m_Mtx);
	for (const auto& rDet : apMsg->detections)
	{
		auto& rEntry = m_mapSeen[rDet.class_name];
		rEntry.first += 1;
		rEntry.second = dNow;
	}
}

/* ------------------------------------------- worker: joint state, reports -- */

void
CROS2PickBridge::TickJointStates()
{
	JointState stSlot;
	if (!m_jointSlot.Read(stSlot))
		return;                     // nothing yet, or every attempt overlapped a write: the next tick is 50 ms away
	const int nDof = stSlot.nDof;
	if (nDof <= 0 || nDof > JS_MAX)
		return;
	// the names of the joints in indy7.urdf; the stamp is needed by
	// robot_state_publisher, which ignores a JointState without one
	sensor_msgs::msg::JointState stMsg;
	stMsg.header.stamp = m_pNode->now();
	for (int i = 0; i < nDof; i++)
	{
		stMsg.name.push_back("joint" + std::to_string(i));
		stMsg.position.push_back(stSlot.adQ[i]);
		stMsg.velocity.push_back(stSlot.adQd[i]);
	}
	m_pPubJoints->publish(stMsg);
}

void
CROS2PickBridge::TickReport()
{
	const double dNow = NowS();
	const double dLast = m_dLastAcceptS.load(std::memory_order_acquire);
	const bool bFresh = dLast >= 0.0 && dNow - dLast <= m_stCfg.dLossS;
	if (bFresh && !m_bHaveTarget)
	{
		double adG[3];
		std::string strClass;
		{
			std::lock_guard<std::mutex> cLock(m_Mtx);
			for (int k = 0; k < 3; k++)
				adG[k] = m_adLastGoal[k];
			strClass = m_strLastClass;
		}
		printf("[bridge] target %s at (%.3f, %.3f, %.3f) m: %s\n",
		       strClass.empty() ? "?" : strClass.c_str(), adG[0], adG[1], adG[2],
		       m_stCfg.pObs != NULL ? "results go into the mailbox" : "goals go into the slot");
		m_bHaveTarget = true;
	}
	else if (!bFresh && m_bHaveTarget)
	{
		const int nWhy = m_nLastReject.load(std::memory_order_relaxed);
		printf("[bridge] target lost: no accepted sample for %.1f s%s%s - the arm stays at the last goal\n",
		       m_stCfg.dLossS, nWhy >= 0 ? "; last rejection: " : "",
		       nWhy >= 0 ? CGoalFilter::Name((CGoalFilter::eVerdict)nWhy) : "");
		m_bHaveTarget = false;
	}
	// rejections of samples that the pipeline marked valid (floor, box, not
	// finite), at most one report per REJECT_PRINT_S; invalid samples (no
	// object, no depth) are reported only through the loss of the target
	if (dNow - m_dRejectShownS < REJECT_PRINT_S)
		return;
	const CGoalFilter::eVerdict aeKinds[3] = {CGoalFilter::eBelowFloor, CGoalFilter::eOutsideBox,
	                                          CGoalFilter::eNotFinite};
	long anNew[3], nNew = 0;
	for (int i = 0; i < 3; i++)
	{
		const long nNow = m_anRejected[aeKinds[i]].load(std::memory_order_relaxed);
		anNew[i] = nNow - m_anRejShown[aeKinds[i]];
		m_anRejShown[aeKinds[i]] = nNow;
		nNew += anNew[i];
	}
	if (nNew > 0)
	{
		printf("[bridge] rejected: %ld with the goal below the floor, %ld outside the box, %ld not finite\n",
		       anNew[0], anNew[1], anNew[2]);
		m_dRejectShownS = dNow;
	}
}

/* --------------------------------------------- worker: the object menu -- */

void
CROS2PickBridge::ShowMenu()
{
	const double dNow = NowS();
	std::vector<std::pair<std::string, int>> vVisible;
	{
		std::lock_guard<std::mutex> cLock(m_Mtx);
		for (auto it = m_mapSeen.begin(); it != m_mapSeen.end();)
		{
			if (dNow - it->second.second > MENU_WINDOW_S)
			{
				it = m_mapSeen.erase(it);
				continue;
			}
			if (it->second.first >= MENU_MIN_COUNT)
				vVisible.push_back({it->first, it->second.first});
			it->second.first = 0;   // the window starts again with every menu
			++it;
		}
	}
	std::sort(vVisible.begin(), vVisible.end());
	char acMenu[1024];
	int nLen = snprintf(acMenu, sizeof(acMenu), "[bridge] classes in view (last %.0f s):\n", MENU_WINDOW_S);
	m_vMenu.clear();
	int nIdx = 1;
	for (const auto& rV : vVisible)
	{
		if (nIdx > 9)               // one digit per entry
			break;
		if (nLen < (int)sizeof(acMenu))
			nLen += snprintf(acMenu + nLen, sizeof(acMenu) - nLen, "  %d %s (%d detections)\n", nIdx, rV.first.c_str(),
			                 rV.second);
		m_vMenu.push_back(rV.first);
		nIdx++;
	}
	// no 'any class' entry: in pick_logic an empty desired_class selects nothing
	if (nLen < (int)sizeof(acMenu))
		nLen += snprintf(acMenu + nLen, sizeof(acMenu) - nLen, "%s  0 cancel; now: %s\n",
		                 vVisible.empty() ? "  (no class in view)\n" : "",
		                 m_strSelected.empty() ? "none" : m_strSelected.c_str());
	fputs(acMenu, stdout);
	m_bMenuOpen.store(true, std::memory_order_release);
}

void
CROS2PickBridge::HandleDigit(int anSel)
{
	if (!m_bMenuOpen.exchange(false))
		return;
	if (anSel == 0)
	{
		printf("[bridge] menu closed\n");
		return;
	}
	if (anSel > (int)m_vMenu.size())
	{
		printf("[bridge] no entry %d - menu closed\n", anSel);
		return;
	}
	const std::string strClass = m_vMenu[anSel - 1];
	if (SetDesiredClass(strClass))
	{
		if (m_stCfg.pObs != NULL && strClass != m_strSelected)
			m_stCfg.pObs->NextTarget();     // the estimate of the control task starts again
		m_strSelected = strClass;
		m_bClassSet.store(true, std::memory_order_release);
		printf("[bridge] desired_class = '%s': the arm follows it\n", strClass.c_str());
		m_bFollowReq.store(true, std::memory_order_release);
	}
}

BOOL
CROS2PickBridge::SetDesiredClass(const std::string& astrClass)
{
	try
	{
		if (!m_pParamClient->service_is_ready() && !m_pParamClient->wait_for_service(1s))
		{
			printf("[bridge] %s does not answer - is the pipeline running?\n", PICK_LOGIC_NODE);
			return FALSE;
		}
		auto cFuture = m_pParamClient->set_parameters({rclcpp::Parameter(PARAM_DESIRED, astrClass)});
		if (cFuture.wait_for(2s) != std::future_status::ready)
		{
			printf("[bridge] setting %s timed out\n", PARAM_DESIRED);
			return FALSE;
		}
		const auto vResults = cFuture.get();
		if (!vResults.empty() && vResults[0].successful)
			return TRUE;
		printf("[bridge] %s refused: %s\n", PARAM_DESIRED, vResults.empty() ? "no result" : vResults[0].reason.c_str());
	}
	catch (const std::exception& e)
	{
		printf("[bridge] setting %s: %s\n", PARAM_DESIRED, e.what());
	}
	return FALSE;
}
