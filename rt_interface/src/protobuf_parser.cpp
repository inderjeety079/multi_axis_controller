#include "protobuf_parser.hpp"

ProtobufParser::ProtobufParser() {
 logger_ = spdlog::get("butler_control_interface")->clone("protobuf_parser");
 logger_fb = spdlog::get("butler_feedback_interface")->clone("protobuf_fb_parser");
 parser_state = HEADER;
}

ProtobufParser::~ProtobufParser() = default;

void ProtobufParser::pack_cmd_msg(const int &axis_id,
   const enum_command_req_msg_type command_type, std::vector<Json::Value>* command,
   std::string *packed_msg) {
       main_control_request_msg proto_mesg;
       proto_mesg.set_msg_type_e_rq(enum_msg_type::CONTROL_REQUEST);
       switch(command_type)
       {
           case enum_command_req_msg_type::JOB_EXECUTE_REQ:
                {
                     //Size of the command vector indicates number of commands
                    for(int cmd_it=0;cmd_it<command->size();cmd_it++)
                    {
                        
                        proto_mesg.set_cmd_req_type_e_rq(command_type);
                        proto_mesg.set_seq_id_u32_rq((*command)[cmd_it]["seq_id"].asInt());
                        proto_mesg.set_timeout_sec_f_rq((*command)[cmd_it]["timeout"].asFloat());

                        submsg_actuator_job *actuator_job = proto_mesg.mutable_actuator_job_sm();
                        //proto_msg.clear_actuator_job_sm();
                        actuator_job->set_actuator_id_u32_rq(axis_id);

                        switch(static_cast<enum_actuatorjobtypes>((*command)[cmd_it]["job_type"].asInt()))
                        {
                            case enum_actuatorjobtypes::MTR_CNTRL_POS_MODE:
                                {
                                    actuator_job->set_job_type_e_rq(static_cast<enum_actuatorjobtypes>((*command)[cmd_it]["job_type"].asInt()));
                                }
                                break;
                            case enum_actuatorjobtypes::MTR_CNTRL_SPEED_MODE:
                                {
                                    actuator_job->set_job_type_e_rq(static_cast<enum_actuatorjobtypes>((*command)[cmd_it]["job_type"].asInt()));
                                    
                                    submsg_md_job_speed_mode *setpoint = actuator_job->mutable_mtr_drv_speed_mode_sm();
                                    setpoint->set_max_rpm_f_rq((*command)[cmd_it]["velocity"].asFloat());
                                }
                                break;
                            default:
                            {
                                logger_->error("Invalid JOB_EXECUTE_REQ type!! {}",(*command)[cmd_it]["job_type"].asInt());
                                return;
                            }

                        }

                        //proto_mesg.set_allocated_actuator_job_sm(&actuator_job); 
                        
                    }
                }
                break;
            case enum_command_req_msg_type::CREATE_ACTUATOR_REQ:
                {
                    proto_mesg.set_cmd_req_type_e_rq(command_type);
                    proto_mesg.set_seq_id_u32_rq(1); 
                    proto_mesg.set_timeout_sec_f_rq(0.1f);
                    submsg_actuator_create_req *actuator_create_req = proto_mesg.mutable_actuator_create_sm();

                    actuator_create_req->set_actuator_id_u32_rq(axis_id);
                    actuator_create_req->set_type_e_rq(enum_actuatortype::MOTOR_DRIVER);
                    actuator_create_req->set_comm_type_e_op(enum_commtype::UART);

                    submsg_mtr_drv_create *actuator_info = actuator_create_req->mutable_actuator_mtr_drv_sm();
                    submsg_uartcomm_data *comm_info = actuator_create_req->mutable_uart_comm_data_sm();

                    // Sanity check command vector
                    if(command->size()==1)
                    {
                        Json::Value command_json = command->front();
                        actuator_info->set_mtr_drv_type_e_rq(parse_motordriver_type(command_json["actuator_params/driver_name"].asString()));
                        actuator_info->set_publish_io_data_b_rq(true);
                        actuator_info->set_drv_channel(std::stoi(command_json["actuator_params/channel"].asString()));
                        actuator_info->set_link_group_id_u32_op(1);                                                    //???????????????????/

                        submsg_wheel_phys_param * phy_info = actuator_info->mutable_phys_param_sm_op();
                        phy_info->set_tick_per_rev_u32_rq(std::stoi(command_json["accessory_params/phys_params/ticks_per_motor_rev"].asString()));
                        phy_info->set_wheel_gear_ratio_f_rq(std::stof(command_json["accessory_params/phys_params/gearbox_ratio"].asString()));
                        phy_info->set_wheel_radium_mm_f_rq(std::stof(command_json["accessory_params/phys_params/wheel_radius"].asString()));

                        comm_info->set_baudrate_u32_rq(std::stoi(command_json["actuator_params/uart/baudrate"].asString()));
                        comm_info->set_uart_num_u32_rq(std::stoi(command_json["actuator_params/uart/channel"].asString()));

                    }
                    else
                    {
                        logger_->error("Packing function received multiple actuator info at once -> Not handled!");
                        return;
                    }

                }
                break;
            case enum_command_req_msg_type::CREATE_SENSOR_REQ:
                {
                    proto_mesg.set_cmd_req_type_e_rq(command_type);
                    proto_mesg.set_seq_id_u32_rq(2);
                    submsg_sensor_create_req *sensor_create_req = proto_mesg.mutable_sensor_create_sm();

                    sensor_create_req->set_actuator_id_u32_rq(axis_id);
                    sensor_create_req->set_sensor_id_u32_rq(axis_id);
                    sensor_create_req->set_type_e_rq(enum_sensortype::ODOM);

                    // Sanity check command vector
                    if(command->size()==1)
                    {
                        Json::Value command_json = command->front();
                        sensor_create_req->set_sensor_pub_freq_u32_rq(std::stoi(command_json["feedback_frequency"].asString()));
                        
                        submsg_odom_create* odom_info = sensor_create_req->mutable_odom_create_sm();
                        odom_info->set_encoder_num_i32_rq(0);                                                     // ??????????
                        
                        submsg_wheel_phys_param * phy_info = odom_info->mutable_phys_param_sm_rq();
                        phy_info->set_tick_per_rev_u32_rq(std::stoi(command_json["accessory_params/phys_params/ticks_per_motor_rev"].asString()));
                        phy_info->set_wheel_gear_ratio_f_rq(std::stof(command_json["accessory_params/phys_params/gearbox_ratio"].asString()));
                        phy_info->set_wheel_radium_mm_f_rq(std::stof(command_json["accessory_params/phys_params/wheel_radius"].asString()));

                    }
                    else
                    {
                        logger_->error("Packing function received multiple sensor info at once -> Not handled!");
                        return;
                    }
   
                }
                break;
            default:
                {}
       }

       // Return protobuf message
       add_header(proto_mesg.ByteSize(),packed_msg);
       if (!proto_mesg.AppendToString(packed_msg)) {
            logger_->error("Failed to serialise proto message!");
            packed_msg->clear();
             return;
        }
        else
        {
            logger_->debug("Packed string is {}",proto_mesg.DebugString());
            logger_->debug("Size of packed string {}, size of serialised mesg {}",packed_msg->size(),proto_mesg.ByteSize());
        }

        /*
       std::string serialised_msg;
        serialised_msg = proto_mesg.SerializeAsString();
        proto_msg.SerializeToString(&serialised_msg);
        if(serialised_msg.empty())
        {
            logger_->error("Protobuf encoding failed!!");
        }
        else
        {
            add_header(proto_mesg.ByteSizeLong(),packed_msg);
            packed_msg->append(serialised_msg);
            logger_->debug("Packed string is {}",proto_mesg.DebugString());
            logger_->debug("Size of packed string {}",packed_msg->size());
        }
        */
        
   }

constexpr int string_hash(const char *s,int len) 
{
    /* Hash function djb2 by Dan Bernstein */
    unsigned long hash = 5381;
    for(int i=0;i<len;i++)
    {
        hash = ((hash << 5) + hash) + s[i];
    }

    return hash;
} 

enum_mtrdrvtype ProtobufParser::parse_motordriver_type(std::string MD_type_string)
{
    // Converts string to appropriate enum
    switch(string_hash(MD_type_string.c_str(),7))
    {
        case string_hash("FBL2360",7):
            return enum_mtrdrvtype::FBL2360;
            break;
        case string_hash("SBL2360",7):
            return enum_mtrdrvtype::SBL2360;
            break;
        case string_hash("HBL2360",7):
            return enum_mtrdrvtype::HBL2360;
            break;
        case string_hash("MBL2360",7):
            return enum_mtrdrvtype::MBL2360;
            break;
        case string_hash("ELMO180",7):
            return enum_mtrdrvtype::ELMO180;
            break;
        case string_hash("ELMO360",7):
            return enum_mtrdrvtype::ELMO360;
            break;
        default:
            logger_->error("Invalid motor driver type received!");
            return enum_mtrdrvtype::FBL2360;
    }
}


bool ProtobufParser::unpack_msg(const std::string &str,
 std::vector<std::map<std::string,std::string>> *key_value_map_vector) {

  const int recvlen = str.size();
  unsigned char* raw_bytes = new unsigned char[recvlen];
  std::copy(str.begin(), str.end(), raw_bytes);

    /* Print received message 
    logger_->debug("Received message is: ");
    for (int i = 0; i < recvlen; i++)
        logger_->debug("0x{0:x}", (int)raw_bytes[i]);
    */
    // std::cout<<" "<<hex<<(int)raw_bytes[i];
    //std::cout << " " << std::endl;

  /* Local Parser variables */
  unsigned char byte;
  size_t num_bytes_stream;
  enum_msg_type message_type;
  unsigned int it = 0;  
  bool result = true;

  if(recvlen==0)
  {
      logger_fb->error("Empty message received!");
      result = false;
  }
  /* Unpack message header params before unpacking key value pairs */

  while(it<recvlen&&result)
  {
      switch (parser_state)
      {
   
        case HEADER:
            {
                unsigned char h1_byte = raw_bytes[it];
                unsigned char h2_byte = raw_bytes[it+1];
                if(h2_byte == COMM_RT_NRT_HEADER_2&&h1_byte==COMM_RT_NRT_HEADER_1)
                {
                    logger_fb->debug("Header field found switching to length!");
                    it = it+1;
                    parser_state = LENGTH;
                }
            }
            break;

        case LENGTH:
            {
                uint8_t upper_byte = raw_bytes[it];
                uint8_t lower_byte = raw_bytes[it+1];
                num_bytes_stream = (upper_byte << 8u) | lower_byte;
                // Check on size of the message 
                if(recvlen-COMM_HEADER_LEN>=num_bytes_stream)
                {
                    logger_fb->debug("Complete message received of {} switching to id!",num_bytes_stream);
                    it = it+1;
                    parser_state = MSG_ID;
                }
                else
                {
                    logger_fb->error("Incomplete message received of size {}/{}",num_bytes_stream,recvlen);
                    //@TODO : Incomplete message handling
                    parser_state = HEADER;
                }
            }
            break;

        case MSG_ID:
            {
                uint8_t upper_byte = raw_bytes[it];
                uint8_t lower_byte = raw_bytes[it+1];
                message_type = static_cast<enum_msg_type>((upper_byte << 8u) | lower_byte);
                //Check on type of message
                if(message_type<=enum_msg_type_MAX&&message_type>=enum_msg_type_MIN)
                {
                    logger_fb->debug("Received message type of {}",message_type); 
                    it = it+1;
                    parser_state = PAYLOAD;
                }
                else
                {
                    logger_fb->error("Invalid message type received!");
                    parser_state = HEADER;
                }
                
            }
            break;

        case PAYLOAD:
            switch(message_type)
            {
                case enum_msg_type::CONTROL_REQUEST:
                    {

                    }
                    break;
                case enum_msg_type::CONTROL_RESPONSE:
                    {
                        main_control_response_msg resp_message;
                        //typecast proto message as string
                        //unsigned char proto_msg[num_bytes_stream] = {0};
                        //memcpy(&proto_msg[0], &raw_bytes[it], num_bytes_stream);
                        
                        //std::string proto_string(reinterpret_cast<char const *>(proto_msg));

                        //Populate protobuf message
                        if (!resp_message.ParseFromArray(raw_bytes+it,num_bytes_stream)) {
                            logger_fb->error("Failed to parse feedback message");
                            result = false;
                        }
                        else
                        {
                            logger_fb->debug("Received message is {}",resp_message.DebugString());        
                            it=it+num_bytes_stream;                
                            // Sanity check message
                            if(resp_message.msg_type_e_rq() == enum_msg_type::CONTROL_RESPONSE)
                            {
                                std::map<std::string,std::string> key_value_map;

                                std::string key = "job_id";
                                std::string value = std::to_string(resp_message.request_seq_id_u32_rq());
                                key_value_map.insert(std::make_pair(key,value));

                                key = "success";
                                value = std::to_string(resp_message.cmd_res_type_e_rq());
                                key_value_map.insert(std::make_pair(key,value));

                                if(resp_message.has_err_code_u32_op())
                                {
                                    key = "err_code";
                                    value = std::to_string(resp_message.err_code_u32_op());
                                    key_value_map.insert(std::make_pair(key,value));
                                }

                                if(resp_message.has_err_source_u32_op())
                                {
                                    key = "err_source";
                                    value = std::to_string(resp_message.err_source_u32_op());
                                    key_value_map.insert(std::make_pair(key,value));
                                }
                                
                                key_value_map_vector->push_back(key_value_map);
                                key_value_map.clear();

                            }
                            else
                            {
                                logger_fb->error("Control response parser received invalid message type!");
                                result = false;
                            }
                        }
                        

                    }
                    break;
                case enum_msg_type::FEEDBACK:
                    {
                        main_feedback_message fb_message;
                        //typecast proto message as string
                        //unsigned char proto_msg[num_bytes_stream] = {0};
                        //memcpy(&proto_msg[0], &raw_bytes[it], num_bytes_stream);
                        //std::string proto_string(reinterpret_cast<char const *>(proto_msg));
                        
                        // Populate protobuf message
                        if (!fb_message.ParseFromArray(raw_bytes+it,num_bytes_stream)) {
                            logger_fb->error("Failed to parse feedback message from array");
                            result = false;
                            }
                        else
                        {
                            it=it+num_bytes_stream;
                            result = unpack_feedback_msg(fb_message,key_value_map_vector);
                        }
                          
                    }
                    break;
                default:
                    ;
            }
            parser_state = HEADER;
            break;

        default:
            logger_fb->debug("ERROR: Default Parser State\n");
            parser_state = HEADER;
      }

      it++;
  }
  
  delete[] raw_bytes;
  return result;
}

bool ProtobufParser::unpack_feedback_msg(main_feedback_message fb_message, \
                    std::vector<std::map<std::string,std::string>> *key_value_map_vector)
{
    // Holds data for each axis
    std::map<std::string,std::string> key_value_map;
    key_value_map.clear();

     // Check if feedback message and count number of axes received in the feedback message and push to container
    if(fb_message.msg_type_e_rq() == enum_msg_type::FEEDBACK && fb_message.has_odom_feedback()&&fb_message.has_md_io_feedback())
    {
        std::vector<axis_container> container;

            if(fb_message.odom_feedback().has_odom_data_1_sm_op() && fb_message.md_io_feedback().has_md_io_data_1_sm_op())
        {
            
            axis_container axis1 = std::make_tuple(fb_message.odom_feedback().odom_data_1_sm_op(), \
                                                        fb_message.md_io_feedback().md_io_data_1_sm_op());
            container.push_back(axis1);
        }
            if(fb_message.odom_feedback().has_odom_data_2_sm_op() && fb_message.md_io_feedback().has_md_io_data_2_sm_op())
        {
            axis_container axis2 = std::make_tuple(fb_message.odom_feedback().odom_data_2_sm_op(), \
                                                        fb_message.md_io_feedback().md_io_data_2_sm_op());
            container.push_back(axis2);
        }
            if(fb_message.odom_feedback().has_odom_data_3_sm_op() && fb_message.md_io_feedback().has_md_io_data_3_sm_op())
        {
            axis_container axis3 = std::make_tuple(fb_message.odom_feedback().odom_data_3_sm_op(), \
                                                        fb_message.md_io_feedback().md_io_data_3_sm_op());
            container.push_back(axis3);
        }
            if(fb_message.odom_feedback().has_odom_data_4_sm_op() && fb_message.md_io_feedback().has_md_io_data_4_sm_op())
        {
            axis_container axis4 = std::make_tuple(fb_message.odom_feedback().odom_data_4_sm_op(), \
                                                        fb_message.md_io_feedback().md_io_data_4_sm_op());
            container.push_back(axis4);
        }

        int num_axes=0;
        // Iterate through all axes in the container and convert data to key value pairs 
        logger_fb->debug("Number of axis containers during parsing: {}",container.size());
        while(num_axes<container.size())
        {
            // Key values are statically named based on the actuator_info header file
            std::string key = "seq_id";
            std::string value = std::to_string(fb_message.seq_id_u32_rq());
            key_value_map.insert(std::make_pair(key,value));

            key = "ts_msec";
            value = std::to_string(fb_message.timestamp_ms_u32_rq());
            key_value_map.insert(std::make_pair(key,value));
            
            key = "sensor_id";
            value = std::to_string(std::get<0>(container[num_axes]).sensor_id_u32_rq());
            key_value_map.insert(std::make_pair(key,value));

            /*
            key = "rel_ticks";
            value = std::to_string(std::get<0>(container[num_axes]).rel_ticks_u32_rq());
            key_value_map.insert(std::make_pair(key,value));
            */

            key = "absolute_ticks";
            value = std::to_string(std::get<0>(container[num_axes]).abs_ticks_u32_rq());
            key_value_map.insert(std::make_pair(key,value));

            key = "current_velocity";
            value = std::to_string(std::get<0>(container[num_axes]).rpm_f_rq());
            key_value_map.insert(std::make_pair(key,value));

            key = "gpio_sensors_state";
            value = std::to_string(std::get<1>(container[num_axes]).io_value_u32_rq());
            key_value_map.insert(std::make_pair(key,value));

            /*
            key = "io_id";
            value = std::to_string(std::get<1>(container[num_axes]).io_id_u32_rq());
            key_value_map.insert(std::make_pair(key,value));
            */

            num_axes++;
            // Insert axis map into vector
            key_value_map_vector->push_back(key_value_map);
            logger_fb->debug("Parsing complete for axis : {}",key_value_map_vector->size());
            key_value_map.clear();
        }
        
    }
    else if(fb_message.msg_type_e_rq() != enum_msg_type::FEEDBACK)
    {
        logger_fb->error("Unpack feedback message function received incorrect proto message {}",fb_message.msg_type_e_rq());
    }
    else
    {
        logger_fb->error("Feedback message is empty!! {} {}",fb_message.has_odom_feedback(),fb_message.has_md_io_feedback());
    }
}


void ProtobufParser::add_header(uint16_t num_bytes,std::string *msg)
{
    int it = 0;
    unsigned char* header = new unsigned char[COMM_HEADER_LEN]; // Length of header

    // Add header bytes
    header[it++]=COMM_NRT_RT_HEADER_1;
    header[it++]=COMM_NRT_RT_HEADER_2;

    // Add length of protobuf message
    header[it++]=(uint8_t)((num_bytes >> 8) & 0xFF); // MSB
    header[it++]=(uint8_t)(num_bytes & 0xFF); //LSB

    // Add type of message
    uint16_t msg_id = enum_msg_type::CONTROL_REQUEST;
    header[it++]=(uint8_t)((msg_id >> 8) & 0xFF); // MSB
    header[it++]=(uint8_t)((msg_id & 0xFF)); //LSB

    //std::string header_string(reinterpret_cast<char const *>(header));

    //*msg = header_string;
    *msg = msg->append((char*)header,it);

}