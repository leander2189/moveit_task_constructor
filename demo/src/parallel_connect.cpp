#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/generate_pose.h>

#include <moveit/task_constructor/cost_terms.h>

using namespace moveit::task_constructor;

/* FixedState - Connect - FixedState */
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto node = rclcpp::Node::make_shared("mtc_tutorial", node_options);
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	Task t;
	t.stages()->setName("parallel connect");
	t.loadRobotModel(node);

	assert(t.getRobotModel()->getName() == "panda");

	auto scene{ std::make_shared<planning_scene::PlanningScene>(t.getRobotModel()) };
	auto& robot_state{ scene->getCurrentStateNonConst() };
	robot_state.setToDefaultValues();
	// robot_state.setToDefaultValues(robot_state.getJointModelGroup("panda_arm"), "transport");
	robot_state.setToRandomPositions(robot_state.getJointModelGroup("panda_arm"));

	auto initial{ std::make_unique<stages::FixedState>("start") };
	initial->setState(scene);
	Stage* current_state_ptr = initial.get();
	t.add(std::move(initial));

	auto pipeline = std::make_shared<solvers::PipelinePlanner>(node);
	auto parallel_container = std::make_unique<Alternatives>("alternatives");

	// First behaviour: move cobot using sampling planner
	{
		auto stage =
		    std::make_unique<stages::Connect>("move", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } });
		parallel_container->add(std::move(stage));
	}

	// Second behaviour: move cobot to parking, move 7th axis and then move cobot back to position
	{
		auto serial_container = std::make_unique<SerialContainer>("combined_move");

		// Move cobot to parking pose
		{
			auto stage = std::make_unique<stages::MoveTo>("move_to_transport", pipeline);
			stage->setGroup("panda_arm");
			stage->setGoal("extended");
			serial_container->add(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::Connect>(
			    "move2", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } });
			// stage->setProperty("merge_mode", stages::Connect::MergeMode::SEQUENTIAL);
			serial_container->add(std::move(stage));
		}

		parallel_container->add(std::move(serial_container));
	}

	auto alternatives{ std::make_unique<Alternatives>("connect") };
	{
		auto connect{ std::make_unique<stages::Connect>(
			 "path length", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } }) };
		connect->setCostTerm(std::make_unique<cost::PathLength>());  // This is the default for Connect, specified for
		                                                             // demonstration purposes
		alternatives->add(std::move(connect));
	}
	{
		auto connect{ std::make_unique<stages::Connect>(
			 "trajectory duration", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } }) };
		connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		alternatives->add(std::move(connect));
	}
	{
		auto connect{ std::make_unique<stages::Connect>(
			 "eef motion", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } }) };
		connect->setCostTerm(std::make_unique<cost::LinkMotion>("panda_hand"));
		alternatives->add(std::move(connect));
	}
	{
		auto connect{ std::make_unique<stages::Connect>(
			 "elbow motion", stages::Connect::GroupPlannerVector{ { "panda_arm", pipeline } }) };
		connect->setCostTerm(std::make_unique<cost::LinkMotion>("panda_link4"));
		alternatives->add(std::move(connect));
	}

	t.add(std::move(parallel_container));
	// t.add(std::move(alternatives));

	/******************************************************
---- *          Generate Place Pose                       *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::GeneratePose>("gen pose");
		stage->properties().set("ik_frame", "panda_link8");
		// Generate Place Pose
		// auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
		// stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		// stage->properties().set("marker_ns", "place_pose");
		// stage->setObject(object);

		// Set target pose
		geometry_msgs::msg::PoseStamped p;
		p.header.frame_id = "panda_link0";
		p.pose.position.x = 0.5;
		p.pose.position.y = 0.5;
		p.pose.position.z = 0.7;
		stage->setPose(p);
		stage->setMonitoredStage(current_state_ptr);  // Hook into attach_object_stage

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(10);
		wrapper->setGroup("panda_arm");
		wrapper->properties().set("eef", "hand");
		// wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
		// wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		t.add(std::move(wrapper));
	}

	/*
	auto goal_scene{ scene->diff() };
	goal_scene->getCurrentStateNonConst().setToDefaultValues(robot_state.getJointModelGroup("panda_arm"), "ready");
	auto goal = std::make_unique<stages::FixedState>("goal");
	goal->setState(goal_scene);
	t.add(std::move(goal));
	*/

	try {
		t.plan(50);
	} catch (const InitStageException& e) {
		std::cout << e << std::endl;
	}

	// keep alive for interactive inspection in rviz
	spinning_thread.join();

	return 0;
}
