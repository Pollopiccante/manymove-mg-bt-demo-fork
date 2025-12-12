// Copyright 2025 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "manymove_cpp_trees/main_imports_helper.hpp"
#include <visualization_msgs/msg/marker.hpp>

std::string buildStreamerBoardAction(std::string action_name, std::string guard_condition_key, std::string action_xml){
  // check if action should run, if so: set the robot_action, after that execute the real action
  std::string check_guard_condition = manymove_cpp_trees::buildConditionXML(guard_condition_key, true);
  // std::stringstream set_robot_action_key_xml;
  // set_robot_action_key_xml << "<SetKeyStringValue key_to_write=\"robot_action\" new_value=\"" << action_name << "\" />";
  std::stringstream seq_name;
  seq_name << action_name << + "_streamer_board_action";

  // sets encountered error key if action failed
  std::stringstream set_encountered_error_xml;
  set_encountered_error_xml << "<SetKeyStringValue key_to_write=\"encountered_error\" new_value=\"" << action_name << "\"/>";
  std::stringstream enc_err_name;
  enc_err_name << "error_check_" << seq_name.str();
  std::string encountered_error_fallback = fallbackWrapperXML(enc_err_name.str(), {action_xml, set_encountered_error_xml.str()});

  return sequenceWrapperXML(seq_name.str(), {check_guard_condition, encountered_error_fallback});
}

std::string createIngredientUpdateXML(std::string ingredient_name){

  std::stringstream name;
  std::stringstream id_key;
  std::stringstream pose_key;
  std::stringstream check_action_name;
  std::stringstream check_update_sequence_name;
  std::stringstream always_true_fallback_name;

  name << "update_" << ingredient_name << "_position";
  id_key << ingredient_name << "_id_key";
  pose_key << ingredient_name << "_pose_key";
  check_action_name << "check_" << ingredient_name;
  check_update_sequence_name << "check_update_sequence_" << ingredient_name;
  always_true_fallback_name << "always_true_fb_" << ingredient_name;

  std::string check_obj_xml = buildObjectActionXML(check_action_name.str(), createCheckObjectExists(id_key.str()));
  std::string update_core_xml = buildObjectActionXML(name.str(), createGetObjectPose(
      id_key.str(), pose_key.str(), "world_frame_key",
      "pick_pre_transform_xyz_rpy_1_key", "pick_post_transform_xyz_rpy_1_key"));
  std::string updateAndCheck = sequenceWrapperXML(check_update_sequence_name.str(), {check_obj_xml, update_core_xml});
  return fallbackWrapperXML(always_true_fallback_name.str(), {updateAndCheck, "<AlwaysSuccess/>"});
}



std::string create_dynamic_object(BT::Blackboard::Ptr bb, std::string name, std::string mesh_key, geometry_msgs::msg::Pose pose, std::vector<double> scale){
  std::stringstream id_key;
  std::stringstream graspable;
  std::stringstream shape_key;
  std::stringstream pose_key;
  std::stringstream scale_key;

  id_key << name << "_id_key";
  graspable << "graspable_" << name;
  shape_key << name << "_shape_key";
  pose_key << name << "_pose_key";
  scale_key << name << "_scale_key";

  bb->set(id_key.str(), graspable.str());
  bb->set(shape_key.str(), "mesh");
  bb->set(pose_key.str(), pose);
  bb->set(scale_key.str(), scale);

  std::stringstream check_action_name;
  std::stringstream add_action_name;
  std::stringstream init_action_name;
  check_action_name << "check_" << name;
  add_action_name << "add_" << name;
  init_action_name << "init_" << name << "obj";

  std::string check_obj_exists_xml = buildObjectActionXML(check_action_name.str(), createCheckObjectExists(id_key.str()));
  std::string check_obj_in_pot = "<CheckObjectInPot obj=\"" + name + "\"/>";
  std::string add_obj_xml = buildObjectActionXML(
    add_action_name.str(), createAddObject(
      id_key.str(), shape_key.str(), "",
      pose_key.str(), scale_key.str(), mesh_key));

  return fallbackWrapperXML(init_action_name.str(), {check_obj_exists_xml, check_obj_in_pot, add_obj_xml});
}

std::string create_ingredient_adder(BT::Blackboard::Ptr bb, std::string ingredient_name, int ingredient_counter) {

  float yOffset = 0.12;
  float xOffset = 0.15;
  int yRows = 3;
  int xCounter = ingredient_counter / yRows;
  int yCounter = ingredient_counter % yRows;

  return create_dynamic_object(
  	bb,
	ingredient_name,
	"ingredients_bowl_key",
	createPoseRPY(-0.3 + xCounter * xOffset, 0.2 + yCounter * yOffset, 0.005, 0.0, 0, 0.0),
	std::vector<double>{0.0025, 0.0025, 0.0025}
  );
}



int main(int argc, char ** argv)
{
  // init rclcpp, node and blackboard
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_client_node");
  auto blackboard = BT::Blackboard::create();
  std::vector<manymove_cpp_trees::BlackboardEntry> keys;
  RobotParams rp = defineRobotParams(node, blackboard, keys); // Define all params and blackboard keys for the robot
  RCLCPP_INFO(node->get_logger(), "BT Client Node started (Purely Programmatic XML).");

  // init move config
  auto move_configs = defineMovementConfigs();
  auto & max_move = move_configs["max_move"];
  max_move.planner_id = "RRTConnectkConfigDefault";
  auto & mid_move = move_configs["mid_move"];
  mid_move.planner_id = "RRTConnectkConfigDefault";
  auto & slow_move = move_configs["slow_move"];
  slow_move.planner_id = "RRTConnectkConfigDefault";


  // ----------------------------------------------------------------------------
  // BlackBoard Initialization
  // ----------------------------------------------------------------------------
  blackboard->set("node", node);
  blackboard->set("my_stop_execution", false);
  // the name of the link to attach the object to, and the object to manipulate
  std::string tcp_frame_name = rp.prefix + rp.tcp_frame;
  blackboard->set("tcp_frame_name_key", tcp_frame_name);
  blackboard->set("object_to_manipulate_key", "graspable_wooden_spoon");
  blackboard->set("touch_links_key", rp.contact_links);

  // blackboard keys that are sent to the backend
  blackboard->set("world_frame_key", "world");
  blackboard->set("robot_action", "Idle");
  blackboard->set("gripper_open", false);
  blackboard->set("temperature", 20.0);
  blackboard->set("heat_level", 0.0); // 0 to 5
  blackboard->set("encountered_error", ""); // 0 to 5
  blackboard->set("pick_target_key", Pose());
  blackboard->set("approach_pick_target_key", Pose());
  blackboard->set("pot_contents", "");

  // pick pose offset keys for ingredients
  blackboard->set("pick_pre_transform_xyz_rpy_1_key", std::vector<double>{0.048+0.02, 0.0, 0.13, 3.1416, 0.0, -0.7854});
  blackboard->set("pick_post_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  blackboard->set("approach_pick_pre_transform_xyz_rpy_1_key",std::vector<double>{-0.15, 0.0, 0.0, 0.0, 1.57, 0.0});

  // mesh keys
  blackboard->set("ingredients_bowl_key", "package://manymove_object_manager/meshes/ingredients_bowl.stl");
  blackboard->set("wooden_spoon_key", "package://manymove_object_manager/meshes/wooden_spoon.stl");
  blackboard->set("empty_key", "package://manymove_object_manager/meshes/cube.stl");
  blackboard->set("cooking_pot_key", "package://manymove_object_manager/meshes/cooking_pot.stl");

  // ----------------------------------------------------------------------------
  // Scene Parameter Definition
  // ----------------------------------------------------------------------------
  std::vector<std::string> ingredient_names = {
	"GroundBeef", "Onions", "Garlic", "Oil", "BellPepper", "TomatoPaste",
	 "CannedTomatoes", "CannedKidneyBeans", "CannedCorn", "Broth", "Spices"
  };
  blackboard->set("manipulation_objects_keys", ingredient_names); // objects to check when to gripper closes

  // ----------------------------------------------------------------------------
  // Initialization Nodes
  // ----------------------------------------------------------------------------

  // [Init Step 1] create node that creates the static objects of the scene: cooking pot and floor
  blackboard->set("ground_id_key", "obstacle_ground");
  blackboard->set("ground_shape_key", "box");
  blackboard->set("ground_dimension_key", std::vector<double>{1.0, 1.0, 0.1});
  blackboard->set("ground_pose_key", createPoseRPY(0.0, 0.0, -0.051, 0.0, 0.0, 0.0));
  blackboard->set("ground_scale_key", std::vector<double>{1.0, 1.0, 1.0});
  std::string check_ground_obj_xml = buildObjectActionXML("check_ground", createCheckObjectExists("ground_id_key"));
  std::string add_ground_obj_xml = buildObjectActionXML("add_ground", createAddObject(
    "ground_id_key", "ground_shape_key", "ground_dimension_key", "ground_pose_key", "ground_scale_key", ""
  ));
  std::string init_ground_obj_xml = fallbackWrapperXML("init_ground_obj", {check_ground_obj_xml, add_ground_obj_xml});
  blackboard->set("cooking_pot_id_key", "obstacle_cooking_pot");
  blackboard->set("cooking_pot_shape_key", "mesh");
  blackboard->set("cooking_pot_pose_key", createPoseRPY(0.0, -0.4, +0.051, 0.0, 0.0, 0.0));
  blackboard->set("cooking_pot_scale_key", std::vector<double>{0.0025, 0.0025, 0.0025});
  std::string check_cooking_pot_obj_xml = buildObjectActionXML("check_cooking_pot", createCheckObjectExists("cooking_pot_id_key"));
  std::string add_cooking_pot_obj_xml = buildObjectActionXML("add_cooking_pot", createAddObject(
      "cooking_pot_id_key", "cooking_pot_shape_key", "", "cooking_pot_pose_key", "cooking_pot_scale_key", "cooking_pot_key"
  ));
  std::string init_cooking_pot_obj_xml = fallbackWrapperXML("init_cooking_pot_obj", {check_cooking_pot_obj_xml, add_cooking_pot_obj_xml});
  std::string spawn_fixed_objects_xml = sequenceWrapperXML("SpawnFixedObjects", {
	init_ground_obj_xml,
	init_cooking_pot_obj_xml
  });

  // [Init Step 2] create nodes that will add the ingredients to the scene,
  // put them all under one ingredient init sequence: init_graspable_ingredients_xml
  std::vector<std::string> ingredient_initializers = {};
  for (int i=0; i<ingredient_names.size(); i++){
    std::string ingredient_name = ingredient_names[i];
	std::string init_ingredient_xml = create_ingredient_adder(blackboard, ingredient_name, i);
	ingredient_initializers.push_back(init_ingredient_xml);
  }
  std::string init_graspable_ingredients_xml = sequenceWrapperXML("SpawnGraspableObjects", ingredient_initializers);

  // [Init Step 3] create node that creates a dummy object attached to the tcp of the panda robot
  // this is used to track the position of the tcp
  std::string init_dummy_tcp_xml = create_dynamic_object(
	blackboard,
	"dummy_tcp",
	"empty_key",
	createPoseRPY(0.25, 0.0, 0.525, 0.0, 0, 0.0), // 0.305, 0.525
	std::vector<double>{0.025, 0.025, 0.025}
  );
  std::string attach_dummy_tcp_xml = buildObjectActionXML(
    "attach_dummy_tcp",
    createAttachObject("dummy_tcp_id_key", "tcp_frame_name_key", "touch_links_key"));
  std::string create_and_attach_dummy_tcp_xml = sequenceWrapperXML("create_and_attach_dummy_tcp", {init_dummy_tcp_xml, attach_dummy_tcp_xml});

  // [Init Step 4] create node that moves the robot to its rest position
  std::vector<double> rest_pos_joints = {0.0, -0.785, 0.0, -2.355, 0.0, 1.57, 0.785};
  std::vector<Move> rest_position = {{rp.prefix, tcp_frame_name, "joint", move_configs["max_move"], "", rest_pos_joints}};
  std::string to_rest_xml = buildMoveXML(rp.prefix, rp.prefix + "toRest", rest_position, blackboard);
  std::string prep_sequence_xml = sequenceWrapperXML(rp.prefix + "ComposedPrepSequence", {to_rest_xml});

  // [Init Combine Steps] complete init sequence
  std::string init_sequence_xml = sequenceWrapperXML("init_sequence", {
	spawn_fixed_objects_xml,
	init_graspable_ingredients_xml,
	create_and_attach_dummy_tcp_xml,
	prep_sequence_xml
  });

  // ----------------------------------------------------------------------------
  // Update Nodes
  // ----------------------------------------------------------------------------
  // [Update Step 1] create node that updates the dummy tcp objects position in the blackboard
  blackboard->set("tcp_pre_transform_xyz_rpy_key", std::vector<double>{-0.102, 0.0, 0.0, 0.0, 1.57, 0.0});
  blackboard->set("tcp_post_transform_xyz_rpy_key", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::string update_dummy_tcp_position_xml = buildObjectActionXML(
	"update_dummy_tcp_position",
	createGetObjectPose(
      "dummy_tcp_id_key", "dummy_tcp_pose_key", "world_frame_key",
      "tcp_pre_transform_xyz_rpy_key", "tcp_post_transform_xyz_rpy_key"
	));

  // [Update Step 2] create BT nodes that will request the position of an ingredient and save it in the blackboard
  // put them all under one ingredient update sequence: update_ingredient_positions_xml
  std::vector<std::string> ingredient_updaters = {};
  for (int i=0; i<ingredient_names.size(); i++){
    std::string ingredient_name = ingredient_names[i];
	std::string update_ingredient_xml = createIngredientUpdateXML(ingredient_name);
	ingredient_updaters.push_back(update_ingredient_xml);
  }
  std::string update_ingredient_positions_xml = sequenceWrapperXML("update_ingredient_positions", ingredient_updaters);

  // [Update Combine Steps] complete update repeater (sequence of above, wrapped in repeater node)
  std::string update_sequence_xml = sequenceWrapperXML("update_sequence", {
	update_dummy_tcp_position_xml,
	update_ingredient_positions_xml
  });
  std::string update_repeater_xml = repeatSequenceWrapperXML("update_repeater", {update_sequence_xml});

  // ----------------------------------------------------------------------------
  // Action Board
  // ----------------------------------------------------------------------------
  // [Action Board Step 1] Define Actions, Wrap them all in a Fallback
  // disable all actions by default
  blackboard->set("streamer_move_to_position_key_bool", false);
  blackboard->set("streamer_close_gripper_key_bool", false);
  blackboard->set("streamer_open_gripper_key_bool", false);
  // ActionDef: Move To Position
  std::vector<Move> move_to_sequence = {{rp.prefix, tcp_frame_name, "pose", move_configs["mid_move"], "approach_pick_target_key"}};
  std::string move_to_move_xml = buildMoveXML(rp.prefix, rp.prefix + "pick", move_to_sequence, blackboard, true);
  std::string move_to_pos_stream = buildStreamerBoardAction("MoveToPosition", "streamer_move_to_position_key_bool", move_to_move_xml);
  // ActionDef: Close Gripper
  std::string attach_obj_xml = buildObjectActionXML("attach_obj_to_manipulate", createAttachObject("object_to_manipulate_key", "tcp_frame_name_key", "touch_links_key"));
  std::string attach_closest_obj_xml = sequenceWrapperXML("attach_closest_obj_to_manipulate", {
	setClosestObjectKeyWrapper("manipulation_objects_keys", "object_to_manipulate_key"), // search all objects that can be manipulated, the closest on is set as object_to_manipulate
	attach_obj_xml // attach current object by object_to_manipulate_key
  });
  std::string move_gripper_close_xml = "<GripperCommandAction position=\"0.014\" max_effort=\"1.0\" action_server=\"" + rp.gripper_action_server + "\"/>";
  std::string close_gripper_sequence_xml = sequenceWrapperXML("CloseGripper_Seq", {
	attach_closest_obj_xml,
	move_gripper_close_xml,
	"<SetKeyBoolValue name=\"setGripperClosed\" robot_prefix=\"hmi_\" key=\"gripper_open\" value=\"false\"/>"
  });
  std::string close_gripper_stream = buildStreamerBoardAction("CloseGripper", "streamer_close_gripper_key_bool", close_gripper_sequence_xml);
  // ActionDef: Open Gripper
  std::string move_gripper_open_xml = "<GripperCommandAction position=\"0.025\" max_effort=\"1.0\" action_server=\"" + rp.gripper_action_server + "\"/>";
  std::string detach_obj_xml = fallbackWrapperXML("detach_obj_to_manipulate_always_success", {
	buildObjectActionXML("detach_obj_to_manipulate", createDetachObject("object_to_manipulate_key", "tcp_frame_name_key")),
	"<AlwaysSuccess />"
  });
  std::string remove_obj_xml = fallbackWrapperXML("detach_obj_to_manipulate_always_success", {
	buildObjectActionXML("remove_obj_to_manipulate", createRemoveObject("object_to_manipulate_key")),
    "<AlwaysSuccess />"
  });

  std::string open_gripper_sequence_xml = sequenceWrapperXML("OpenGripper_Seq", {
	move_gripper_open_xml,
	detach_obj_xml,
	"<AddPotContentNode name=\"addToPot\" />",
	remove_obj_xml,
	"<SetKeyBoolValue name=\"setGripperOpen\" robot_prefix=\"hmi_\" key=\"gripper_open\" value=\"true\"/>"
  });
  std::string open_gripper_stream = buildStreamerBoardAction("OpenGripper", "streamer_open_gripper_key_bool", open_gripper_sequence_xml);
  // combine actions in action board fallback
  std::string streamer_board = reactiveFBWrapperXML("StreamerBoardFallback", {close_gripper_stream, open_gripper_stream, move_to_pos_stream, "<AlwaysPending resetRobotAction=\"true\" name=\"IdleForever\"/>"});

  // [Action Board Step 2] Create Node to reset trigger keys after successful action
  std::string resets1 = sequenceWrapperXML("check1", {manymove_cpp_trees::buildConditionXML("streamer_move_to_position_key_bool", true), "<SetKeyBoolValue name=\"set1\" robot_prefix=\"hmi_\" key=\"streamer_move_to_position_key_bool\" value=\"false\"/>"});
  std::string resets2 = sequenceWrapperXML("check2", {manymove_cpp_trees::buildConditionXML("streamer_close_gripper_key_bool", true), "<SetKeyBoolValue name=\"set2\" robot_prefix=\"hmi_\" key=\"streamer_close_gripper_key_bool\" value=\"false\"/>"});
  std::string resets3 = sequenceWrapperXML("check3", {manymove_cpp_trees::buildConditionXML("streamer_open_gripper_key_bool", true), "<SetKeyBoolValue name=\"set3\" robot_prefix=\"hmi_\" key=\"streamer_open_gripper_key_bool\" value=\"false\"/>"});
  std::stringstream succ_resets1;
  std::stringstream succ_resets2;
  std::stringstream succ_resets3;
  succ_resets1 << "<ForceSuccess>" << resets1 << "</ForceSuccess>";
  succ_resets2 << "<ForceSuccess>" << resets2 << "</ForceSuccess>";
  succ_resets3 << "<ForceSuccess>" << resets3 << "</ForceSuccess>";
  std::string reset_stream_actions = sequenceWrapperXML("ResetStreamActions", {succ_resets1.str(), succ_resets2.str(), succ_resets3.str(), "<AlwaysSuccess />"});

  // [Action Board Combine Steps] Resetable Action Board
  std::string resetable_action_board_xml = reactiveWrapperXML("resetable_action_board", {streamer_board, reset_stream_actions});

  // ----------------------------------------------------------------------------
  // Forced Error Board
  // ----------------------------------------------------------------------------
  // [Forced Error Board Step 1] Define Forced Error Actions, Wrap them all in a Fallback
  // disable all by default
  blackboard->set("error_drop_object_key_bool", false);
  // ErrorActionDef: Drop Object
  std::string drop_object_sequence_xml = sequenceWrapperXML("OpenGripper_Seq", {
	detach_obj_xml,
	remove_obj_xml,
	"<DropObject />", // custom node setting  bb->get("object_to_manipulate_key")  + _pose_key = current pos - z value
	init_graspable_ingredients_xml, // re-initialize all
  });
  std::string drop_object_succeed_fallback = fallbackWrapperXML("succeed_drop", {drop_object_sequence_xml, "<AlwaysSuccess />"});
  std::string drop_object_error = buildStreamerBoardAction("DropObject", "error_drop_object_key_bool", drop_object_succeed_fallback);
  // combine actions in action board fallback
  std::string error_board = reactiveFBWrapperXML("ErrorBoardFallback", {drop_object_error, "<AlwaysPending resetRobotAction=\"false\" name=\"IdleForeverError\"/>"});
  // [Forced Error Board Step 2] Create Node to reset trigger keys after successful error action
  std::string error_resets1 = sequenceWrapperXML("checkError1", {manymove_cpp_trees::buildConditionXML("error_drop_object_key_bool", true), "<SetKeyBoolValue name=\"seterror1\" robot_prefix=\"hmi_\" key=\"error_drop_object_key_bool\" value=\"false\"/>"});
  std::stringstream succ_resets_error1;
  succ_resets_error1 << "<ForceSuccess>" << error_resets1 << "</ForceSuccess>";
  std::string reset_error_actions = sequenceWrapperXML("ResetErrorActions", {succ_resets_error1.str(), "<AlwaysSuccess />"});
  // [Forced Error Board Combine Steps] Resetable Error Action Board
  std::string resetable_error_action_board = reactiveWrapperXML("ErrorBoardResetter", {error_board, reset_error_actions}); // ,reset_error_actions

  // ----------------------------------------------------------------------------
  // Construct Root Node
  // ----------------------------------------------------------------------------
  // Parallel node to trigger action board, error board, backend communication and update repeater
  // the node resets whenever one of the boards succeedes
  std::string parallel_streamer_and_updates_xml = parallelWrapperXML("ParallelStreamerAndUpdates", {
	resetable_action_board_xml,
	resetable_error_action_board,
	"<BackendCommunicationNode name=\"BackendComm\"/>",
	update_repeater_xml
  }, 1, 4);
  // Repeat and retry node to tick the streamer board forever
  std::string repeat_forever_wrapper_xml = repeatSequenceWrapperXML("RepeatForever", {parallel_streamer_and_updates_xml}, -1);
  std::string retry_forever_wrapper_xml = retrySequenceWrapperXML("CycleUntilSuccess", {repeat_forever_wrapper_xml}, -1);
  // GlobalMasterSequence set up the scene once, afterwards tick the streamer board forever
  std::string master_body = sequenceWrapperXML("GlobalMasterSequence", {init_sequence_xml, retry_forever_wrapper_xml});

  // ----------------------------------------------------------------------------
  // Run the Tree: Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
  // ----------------------------------------------------------------------------
  std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

  RCLCPP_INFO(
    node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

  // Register node types
  BT::BehaviorTreeFactory factory;
  registerAllNodeTypes(factory);

  // Create the tree from final_tree_xml
  BT::Tree tree;
  try {
    tree = factory.createTreeFromText(final_tree_xml, blackboard);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create tree: %s", ex.what());
    return 1;
  }

  // ZMQ publisher (optional, to visualize in Groot)
  BT::PublisherZMQ publisher(tree);

  // Create the HMI Service Node and pass the same blackboard
  auto hmi_node =
    std::make_shared<manymove_cpp_trees::HMIServiceNode>("hmi_service_node", blackboard, keys);
  RCLCPP_INFO(node->get_logger(), "HMI Service Node instantiated.");

  // Create a MultiThreadedExecutor so that both nodes can be spun concurrently.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(hmi_node);

  // Tick the tree in a loop.
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    executor.spin_some();
    BT::NodeStatus status = tree.tickRoot();

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "BT ended SUCCESS.");
      break;
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "BT ended FAILURE.");
      break;
    }
    rate.sleep();
  }

  tree.rootNode()->halt();
  rclcpp::shutdown();
  return 0;
}
