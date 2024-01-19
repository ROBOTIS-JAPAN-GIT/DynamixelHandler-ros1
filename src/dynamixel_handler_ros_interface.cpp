#include "dynamixel_handler.hpp"

using namespace dyn_x;

string update_info(const vector<uint8_t>& id_list, const string& what_updated) {
    char header[99]; 
    sprintf(header, "[%d] servo(s) %s are updated", (int)id_list.size(), what_updated.c_str());
    return id_list_layout(id_list, string(header));
}

vector<uint8_t> store_cmd(
    const vector<uint16_t>& id_list, const vector<double>& cmd_list, bool is_angle,
    DynamixelHandler::CmdValueIndex cmd_index, 
    pair<DynamixelHandler::OptLimitIndex, DynamixelHandler::OptLimitIndex> lim_index
) {
    vector<uint8_t> store_id_list; 
    if ( id_list.size() != cmd_list.size() ) return store_id_list;
    for (size_t i=0; i<id_list.size(); i++){
        uint8_t id = id_list[i];
        auto value = is_angle ? deg2rad(cmd_list[i]) : cmd_list[i];
        auto& limit = DynamixelHandler::option_limit_[id];
        auto val_max = ( DynamixelHandler::NONE == lim_index.second ) ?  256*2*M_PI : limit[lim_index.second]; //このあたり一般性のない書き方していてキモい
        auto val_min = ( DynamixelHandler::NONE == lim_index.first  ) ? -256*2*M_PI :
                       (        lim_index.first == lim_index.second ) ?   - val_max : limit[lim_index.first];
        DynamixelHandler::cmd_values_[id][cmd_index] = clamp( value, val_min, val_max );
        DynamixelHandler::is_cmd_updated_[id] = true;
        DynamixelHandler::list_write_cmd_.insert(cmd_index);
        store_id_list.push_back(id);
    }
    return store_id_list;
}

//* ROS関係

void DynamixelHandler::CallBackDxlCommand(const dynamixel_handler::DynamixelCommand& msg) {
    vector<uint8_t> id_list;
    if ( msg.id_list.empty() || msg.id_list[0]==0xFE) for (auto id : id_list_) id_list.push_back(id);
                                                 else for (auto id : msg.id_list) id_list.push_back(id);
    char header[100]; sprintf(header, "Command [%s] \n (id_list=[] or [254] means all IDs)", msg.command.c_str());
    ROS_INFO_STREAM(id_list_layout(id_list, string(header)));
    if (msg.command == "clear_error" || msg.command == "CE")
        for (auto id : id_list) { ClearHardwareError(id); TorqueOn(id);}
    if (msg.command == "torque_on"   || msg.command == "TON") 
        for (auto id : id_list) TorqueOn(id);
    if (msg.command == "torque_off"  || msg.command == "TOFF")
        for (auto id : id_list) TorqueOff(id);
    if (msg.command == "enable") 
        for (auto id : id_list) WriteTorqueEnable(id, true);
    if (msg.command == "disable")
        for (auto id : id_list) WriteTorqueEnable(id, false);
    if (msg.command == "reboot") 
        for (auto id : id_list) dyn_comm_.Reboot(id);
}

void DynamixelHandler::CallBackDxlCmd_X_Position(const dynamixel_handler::DynamixelCommand_X_ControlPosition& msg) {
    for ( const uint8_t& id : msg.id_list ) ChangeOperatingMode(id, OPERATING_MODE_POSITION);
    vector<uint8_t> stored_pos = store_cmd( msg.id_list, msg.position__deg, true,
                                            GOAL_POSITION, {MIN_POSITION_LIMIT, MAX_POSITION_LIMIT} );
    if (varbose_callback_ && !stored_pos.empty()) ROS_INFO_STREAM(update_info(stored_pos, "goal_position"));
    vector<uint8_t> stored_pv = store_cmd(  msg.id_list, msg.profile_vel__deg_s, true,
                                            PROFILE_VEL, {VELOCITY_LIMIT, VELOCITY_LIMIT} );
    if (varbose_callback_ && !stored_pv.empty()) ROS_INFO_STREAM(update_info(stored_pv, "profile_velocity"));
    vector<uint8_t> stored_pa = store_cmd(  msg.id_list, msg.profile_acc__deg_ss, true,
                                            PROFILE_ACC, {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
    if (varbose_callback_ && !stored_pa.empty()) ROS_INFO_STREAM(update_info(stored_pa, "profile_acceleration"));
    if ( stored_pos.empty() && stored_pv.empty() && stored_pa.empty() ) 
        ROS_ERROR("Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_Velocity(const dynamixel_handler::DynamixelCommand_X_ControlVelocity& msg) {
    for ( const uint8_t& id : msg.id_list ) ChangeOperatingMode(id, OPERATING_MODE_VELOCITY);
    vector<uint8_t> stored_vel = store_cmd( msg.id_list, msg.velocity__deg_s, true,
                                            GOAL_VELOCITY, {VELOCITY_LIMIT, VELOCITY_LIMIT} );
    if (varbose_callback_ && !stored_vel.empty()) ROS_INFO_STREAM(update_info(stored_vel, "goal_velocity"));
    vector<uint8_t> stored_p = store_cmd(   msg.id_list, msg.profile_acc__deg_ss, true,
                                            PROFILE_ACC, {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
    if (varbose_callback_ && !stored_p.empty()) ROS_INFO_STREAM(update_info(stored_p, "profile_acceleration"));
    if ( stored_vel.empty() && stored_p.empty() ) 
        ROS_ERROR("Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_Current(const dynamixel_handler::DynamixelCommand_X_ControlCurrent& msg) {
    for ( const uint8_t& id : msg.id_list ) ChangeOperatingMode(id, OPERATING_MODE_CURRENT);
    vector<uint8_t> stored_cur = store_cmd( msg.id_list, msg.current__mA, false,
                                            GOAL_CURRENT, {CURRENT_LIMIT, CURRENT_LIMIT} );
    if (varbose_callback_ && !stored_cur.empty()) ROS_INFO_STREAM(update_info(stored_cur, "goal_current"));
    if ( stored_cur.empty() ) ROS_ERROR("Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_CurrentPosition(const dynamixel_handler::DynamixelCommand_X_ControlCurrentPosition& msg) {
    for ( const uint8_t& id : msg.id_list ) ChangeOperatingMode(id, OPERATING_MODE_CURRENT_BASE_POSITION); 
    vector<double> ext_pos(max(msg.position__deg.size(), msg.rotation.size()), 0.0);
    for (size_t i=0; i<ext_pos.size(); i++) ext_pos[i] = (msg.position__deg.size() == ext_pos.size() ? msg.position__deg[i] : 0.0 )
                                                            + (msg.rotation.size() == ext_pos.size() ? msg.rotation[i]*360  : 0.0 );
    vector<uint8_t> stored_pos = store_cmd(  msg.id_list, ext_pos, true,
                                             GOAL_POSITION, {NONE, NONE} );
    if (varbose_callback_ && !stored_pos.empty()) ROS_INFO_STREAM(update_info(stored_pos, "goal_position"));
    vector<uint8_t> stored__cur = store_cmd( msg.id_list, msg.current__mA, false,
                                             GOAL_CURRENT, {CURRENT_LIMIT, CURRENT_LIMIT} );
    if (varbose_callback_ && !stored__cur.empty()) ROS_INFO_STREAM(update_info(stored__cur, "goal_current"));
    vector<uint8_t> stored_pv = store_cmd(   msg.id_list, msg.profile_vel__deg_s, true,
                                             PROFILE_VEL, {VELOCITY_LIMIT, VELOCITY_LIMIT} );
    if (varbose_callback_ && !stored_pv.empty()) ROS_INFO_STREAM(update_info(stored_pv, "profile_velocity"));
    vector<uint8_t> stored_pa = store_cmd(   msg.id_list, msg.profile_acc__deg_ss, true,
                                             PROFILE_ACC, {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
    if (varbose_callback_ && !stored_pa.empty()) ROS_INFO_STREAM(update_info(stored_pa, "profile_acceleration"));
    if ( stored_pos.empty() && stored__cur.empty() && stored_pv.empty() && stored_pa.empty() ) 
        ROS_ERROR("Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_ExtendedPosition(const dynamixel_handler::DynamixelCommand_X_ControlExtendedPosition& msg) {
    for ( const uint8_t& id : msg.id_list ) ChangeOperatingMode(id, OPERATING_MODE_EXTENDED_POSITION);
    vector<double> ext_pos(max(msg.position__deg.size(), msg.rotation.size()), 0.0);
    for (size_t i=0; i<ext_pos.size(); i++) ext_pos[i] = (msg.position__deg.size() == ext_pos.size() ? msg.position__deg[i] : 0.0 )
                                                            + (msg.rotation.size() == ext_pos.size() ? msg.rotation[i]*360  : 0.0 );
    vector<uint8_t> stored_pos = store_cmd( msg.id_list, ext_pos, true,
                                            GOAL_POSITION, {NONE, NONE} );
    if (varbose_callback_ && !stored_pos.empty()) ROS_INFO_STREAM(update_info(stored_pos, "goal_position"));
    vector<uint8_t> stored_pv = store_cmd( msg.id_list, msg.profile_vel__deg_s, true,
                                            PROFILE_VEL, {VELOCITY_LIMIT, VELOCITY_LIMIT} );
    if (varbose_callback_ && !stored_pv.empty()) ROS_INFO_STREAM(update_info(stored_pv, "profile_velocity"));
    vector<uint8_t> stored_pa = store_cmd( msg.id_list, msg.profile_acc__deg_ss, true,
                                            PROFILE_ACC, {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
    if (varbose_callback_ && !stored_pa.empty()) ROS_INFO_STREAM(update_info(stored_pa, "profile_acceleration"));
    if ( stored_pos.empty() && stored_pv.empty() && stored_pa.empty() ) 
        ROS_ERROR("Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlOpt_Gain(const dynamixel_handler::DynamixelOption_Gain& msg) {
    // if (varbose_callback_) ROS_INFO("CallBackDxlOpt_Gain");
    bool is_any = false;
    if (msg.id_list.size() == msg.velocity_i_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.velocity_p_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_d_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_i_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_p_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.feedforward_acc_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.feedforward_vel_gain__pulse.size()){ is_any=true;}
    if (varbose_callback_) {
        //  if (is_any) ROS_INFO(" - %d servo(s) gain are updated", (int)msg.id_list.size());
        //  else                  ROS_ERROR("Element size all dismatch; skiped callback");
    }
}

void DynamixelHandler::CallBackDxlOpt_Limit(const dynamixel_handler::DynamixelOption_Limit& msg) {
    // if (varbose_callback_) ROS_INFO("CallBackDxlOpt_Limit");
    bool is_any = false;

    if (msg.id_list.size() == msg.temperature_limit__degC.size()   ){is_any=true;}
    if (msg.id_list.size() == msg.max_voltage_limit__V.size()      ){is_any=true;}
    if (msg.id_list.size() == msg.min_voltage_limit__V.size()      ){is_any=true;}
    if (msg.id_list.size() == msg.pwm_limit__percent.size()        ){is_any=true;}
    if (msg.id_list.size() == msg.current_limit__mA.size()         ){is_any=true;}
    if (msg.id_list.size() == msg.acceleration_limit__deg_ss.size()){is_any=true;}
    if (msg.id_list.size() == msg.velocity_limit__deg_s.size()     ){is_any=true;}
    if (msg.id_list.size() == msg.max_position_limit__deg.size()   ){is_any=true;}
    if (msg.id_list.size() == msg.min_position_limit__deg.size()   ){is_any=true;}

    if (varbose_callback_) {
        //  if (is_any) ROS_INFO(" - %d servo(s) limit are updated", (int)msg.id_list.size());
        //  else                  ROS_ERROR("Element size all dismatch; skiped callback");
    }
}

void DynamixelHandler::CallBackDxlOpt_Mode(const dynamixel_handler::DynamixelOption_Mode& msg) {
 
}

double round4(double val) { return round(val*10000.0)/10000.0; }

void DynamixelHandler::BroadcastDxlState(){
    dynamixel_handler::DynamixelState msg;
    msg.stamp = ros::Time::now();
    for (const auto& [id, value] : state_values_) {
        msg.id_list.push_back(id);
        for (auto state : list_read_state_) switch(state) {
            case PRESENT_PWM:          msg.pwm__percent.push_back         (round4(value[state]    )); break;
            case PRESENT_CURRENT:      msg.current__mA.push_back          (round4(value[state]    )); break;
            case PRESENT_VELOCITY:     msg.velocity__deg_s.push_back      (round4(value[state]/DEG)); break;
            case PRESENT_POSITION:     msg.position__deg.push_back        (round4(value[state]/DEG)); break;
            case VELOCITY_TRAJECTORY:  msg.vel_trajectory__deg_s.push_back(round4(value[state]/DEG)); break;
            case POSITION_TRAJECTORY:  msg.pos_trajectory__deg.push_back  (round4(value[state]/DEG)); break;
            case PRESENT_TEMPERTURE:   msg.temperature__degC.push_back    (round4(value[state]    )); break;
            case PRESENT_INPUT_VOLTAGE:msg.input_voltage__V.push_back     (round4(value[state]    )); break;
        }
    }
    pub_state_.publish(msg);
}

void DynamixelHandler::BroadcastDxlError(){
    dynamixel_handler::DynamixelError msg;
    msg.stamp = ros::Time::now();
    for (const auto& [id, error]: hardware_error_) {
        if (error[INPUT_VOLTAGE     ]) msg.input_voltage.push_back     (id);
        if (error[MOTOR_HALL_SENSOR ]) msg.motor_hall_sensor.push_back (id);
        if (error[OVERHEATING       ]) msg.overheating.push_back       (id);
        if (error[MOTOR_ENCODER     ]) msg.motor_encoder.push_back     (id);
        if (error[ELECTRONICAL_SHOCK]) msg.electronical_shock.push_back(id);
        if (error[OVERLOAD          ]) msg.overload.push_back          (id);
    }
    pub_error_.publish(msg);
}

void DynamixelHandler::BroadcastDxlOpt_Limit(){
    dynamixel_handler::DynamixelOption_Limit msg;
    msg.stamp = ros::Time::now();
    for (const auto& [id, limit] : option_limit_) {
        msg.id_list.push_back(id);
        msg.temperature_limit__degC.push_back   (round4(limit[TEMPERATURE_LIMIT ]));
        msg.max_voltage_limit__V.push_back      (round4(limit[MAX_VOLTAGE_LIMIT ]));
        msg.min_voltage_limit__V.push_back      (round4(limit[MIN_VOLTAGE_LIMIT ]));
        msg.pwm_limit__percent.push_back        (round4(limit[PWM_LIMIT         ]));
        msg.current_limit__mA.push_back         (round4(limit[CURRENT_LIMIT     ]));
        msg.acceleration_limit__deg_ss.push_back(round4(limit[ACCELERATION_LIMIT]/DEG));
        msg.velocity_limit__deg_s.push_back     (round4(limit[VELOCITY_LIMIT    ]/DEG));
        msg.max_position_limit__deg.push_back   (round4(limit[MAX_POSITION_LIMIT]/DEG));
        msg.min_position_limit__deg.push_back   (round4(limit[MIN_POSITION_LIMIT]/DEG));
    }
    pub_opt_limit_.publish(msg);
}

void DynamixelHandler::BroadcastDxlOpt_Gain(){
    dynamixel_handler::DynamixelOption_Gain msg;
    msg.stamp = ros::Time::now();
    for ( const auto& [id, gain] : option_gain_ ) {
        msg.id_list.push_back(id);
        msg.velocity_i_gain__pulse.push_back     (gain[VELOCITY_I_GAIN     ]);
        msg.velocity_p_gain__pulse.push_back     (gain[VELOCITY_P_GAIN     ]);
        msg.position_d_gain__pulse.push_back     (gain[POSITION_D_GAIN     ]);
        msg.position_i_gain__pulse.push_back     (gain[POSITION_I_GAIN     ]);
        msg.position_p_gain__pulse.push_back     (gain[POSITION_P_GAIN     ]);
        msg.feedforward_acc_gain__pulse.push_back(gain[FEEDFORWARD_ACC_GAIN]);
        msg.feedforward_vel_gain__pulse.push_back(gain[FEEDFORWARD_VEL_GAIN]);
    }
    pub_opt_gain_.publish(msg);
}

void DynamixelHandler::BroadcastDxlOpt_Mode(){
    dynamixel_handler::DynamixelOption_Mode msg;
    msg.stamp = ros::Time::now();
    for ( const auto& id : id_list_ ) {
        msg.id_list.push_back(id);
        msg.torque_enable.push_back(tq_mode_[id]);
        switch(op_mode_[id]) {
            case OPERATING_MODE_CURRENT:              msg.operating_mode.push_back("current");           break;
            case OPERATING_MODE_VELOCITY:             msg.operating_mode.push_back("velocity");          break;
            case OPERATING_MODE_POSITION:             msg.operating_mode.push_back("position");          break;
            case OPERATING_MODE_EXTENDED_POSITION:    msg.operating_mode.push_back("extended_position"); break;
            case OPERATING_MODE_CURRENT_BASE_POSITION:msg.operating_mode.push_back("current_position");  break;
        }
        switch(dv_mode_[id]) {
            default: msg.drive_mode.push_back("unknown"); break;
        }
    }
    pub_opt_mode_.publish(msg);
}

void DynamixelHandler::BroadcastDxlOpt_Goal(){
    dynamixel_handler::DynamixelOption_Goal msg;
    msg.stamp = ros::Time::now();
    for ( const auto& [id, goal] : option_goal_ ) {
        msg.id_list.push_back(id);
        msg.pwm__percent.push_back       (round4(goal[GOAL_PWM         ]));
        msg.current__mA.push_back        (round4(goal[GOAL_CURRENT     ]));
        msg.velocity__deg_s.push_back    (round4(goal[GOAL_VELOCITY    ]/DEG));
        msg.profile_vel__deg_s.push_back (round4(goal[PROFILE_VEL      ]/DEG));
        msg.profile_acc__deg_ss.push_back(round4(goal[PROFILE_ACC      ]/DEG));
        msg.position__deg.push_back      (round4(goal[GOAL_POSITION    ]/DEG));
    }
    pub_opt_goal_.publish(msg);
}