
#include <iostream>
#include <chrono>
#include "rt_interface.hpp"


/**
 * \brief Construct a new Rt Interface:: Rt Interface object
 * 
 * \param robot_config_file_path 
 */
RtInterface::RtInterface(std::string config_file_path,std::string connection_name,std::string on_connect_msg_):
            Interface(config_file_path, connection_name, on_connect_msg_)
{
  /**
   * \brief get the pointer to tcp server object of rt_axes_control
   * connection
   */
  logger_ = spdlog::get("butler_control_interface")->clone("rt_interface");
  logger_fb = spdlog::get("butler_feedback_interface")->clone("rt_interface_fb");
  config_parser_ptr_ = ActuatorConfigParser::get_unique_instance(" ");

  /* Retrieve Actuator map from config server*/
  actuator_map_ = config_parser_ptr_->get_actuator_map();


  /* Get RT Control and Feedback Interfaces*/
  control_interface_ = 
    connection_manager_->get_connection_handle("rt_axes_controls");

  feedback_interface_ =
    connection_manager_->get_connection_handle("rt_axes_feedback");


  /* Create Protobuf Parser for message serialization and deserialization*/
  logger_->debug("Creating Protobuf Parser");
  try {
      protobuf_parser_ = std::make_unique<ProtobufParser> ();
  } catch (std::bad_alloc &ba) {

      std::cerr<< "bad allocation while creating protobuf parser: " <<
       ba.what();
  }

  /* Initialize the message sequence ID of both interfaces */
  control_iface_seq_id_ = 0;
  feedback_iface_seq_id_ = 0;

  for (auto &element:actuator_map_) {
    std::string axis_name = element.first;
    feedback_data_state_map_.emplace(axis_name,false);
  }
}

/* Spawns threads for listening to control and feedback interfaces
 * data.*/

void RtInterface::on_init() {
// TODO(Inderjeet): Add the intialization here

  logger_->debug("Connection with RT System Established");

  if (get_control_iface_conn_status()) {
    control_iface_listener_ = std::thread(
      &RtInterface::parse_control_iface_msg_new, this);
  }

  if(get_feedback_iface_conn_status()) {

    feedback_iface_listener_ = std::thread(
      &RtInterface::parse_feedback_iface_msg_new, this);

    update_feedback_thread_ = std::thread(&RtInterface::update_feedback,
      this);


  }
}

/**
 * \brief Destroy the Rt Interface:: Rt Interface object0
 * 
 */
RtInterface::~RtInterface() {

}


void RtInterface::join_listener_threads(void) {

  control_iface_listener_.join();
  feedback_iface_listener_.join();

}

void RtInterface::retrieve_control_iface_msg(std::string *msg) {
    control_interface_->retreive_message(msg);
}

/* Thread handler which listens to control interface data and unpacks it*/
void RtInterface::parse_control_iface_msg_new() {

  bool unpack_status;
  std::string raw_string;
  std::vector<std::map<std::string,std::string>> info_key_value_map_vector;
  std::map<std::string,std::string> info_key_value_map;

  auto start_parsing_time = std::chrono::high_resolution_clock::now();

  while (get_rt_interface_conn_status()) {
    unpack_status = false;
    retrieve_control_iface_msg(&raw_string);
    logger_fb->debug("Control Iface msg: {}", raw_string);
  
    // Deserialise rawstring into map of key value pairs using protobuf
    unpack_status = protobuf_parser_->unpack_msg(raw_string, &info_key_value_map_vector); 

     if(unpack_status)
      {
        // size of the vector indicates number of control resp messages received
        int num_msg = info_key_value_map_vector.size();
        for(int i=0;i<num_msg;i++)
        {
          info_key_value_map = info_key_value_map_vector.back();
          info_key_value_map_vector.pop_back();
        
          unpack_status = update_job_status_new(info_key_value_map);
          if(!unpack_status)
            {
              logger_fb->error("Exception in updating job status!");
              break;
            }

        }
      }

    if (unpack_status ) {

      auto completed_parsing_time = std::chrono::high_resolution_clock::now();

      std::chrono::microseconds dt_p =
        std::chrono::duration_cast<std::chrono::microseconds> (
          completed_parsing_time - start_parsing_time);

      logger_fb->debug("Control parsing successful, dT_P: {} us , string buffer size: {}",
        dt_p.count(), 0);


    } else {
      logger_fb->debug("Control parsing unsuccessful. continue.. ");
    }

  }  

}

/* Deprecated function*/
void RtInterface::parse_control_iface_msg() {
  std::string raw_string;
  std::string packed_msg;
  std::vector<std::string> msg_substrings;
  bool unpack_status;

  while (get_rt_interface_conn_status()) {

    unpack_status = false;
    retrieve_control_iface_msg(&raw_string);
    logger_->debug("Control Iface msg: {}", raw_string);

    auto start_parsing_time = std::chrono::high_resolution_clock::now();
//    std::size_t msg_delim_pos_carr_ret = packed_msg.find('\r');

    std::vector<std::string>().swap(msg_substrings);
    raw_string.erase(0,1);
    packed_msg = "{seq_id=1&ts_seconds=1626&ts_msec=785&" + raw_string;
    boost::split(msg_substrings, packed_msg, [](char c) { return c == '\n';});
    logger_->debug("job status after split :");

    for (auto &substring: msg_substrings) {
      logger_->debug("{}",substring);
    }
    packed_msg.clear();

    for (auto &substring: msg_substrings) {

      std::size_t head;
      std::string substr = substring;
      std::size_t tail = substr.find(ProtocolParser::message_delim_);
      std::string control_iface_payload;

      logger_->debug("rt control iface msg substring: {}",substring);

      while (tail != std::string::npos) {

        head = substr.find(ProtocolParser::header_);
        if(std::string::npos != head) {

          if (head < tail) {
            /*remove head and tail*/
            control_iface_payload = substr.substr(head + 1, tail - 1);

            logger_->debug("Axis Feedback Msg: {}", control_iface_payload);
            unpack_status = unpack_control_iface_msg(control_iface_payload);
            substr.erase(head, tail + 2);
          } else {
            substr.erase(0, tail + 2);
          }
        }

        else {
          substr.erase(0, tail+3);
        }

        tail = substr.find(ProtocolParser::message_delim_);
      }
    }

    if (unpack_status) {


    }

  }
}


void RtInterface::send_control_iface_msg(const std::string& str) {
    /*Push the string to txq for transmitting it to RT system*/
    control_interface_ =
        connection_manager_->get_connection_handle("rt_axes_controls");
    logger_->debug("Message to be transmitted: {}", str);
//    control_interface_->txq->push(str.c_str());
    control_interface_->send_message(str);
}

void RtInterface::retrieve_feedback_iface_msg(std::string *msg) {
    feedback_interface_->retreive_message(msg);
}

/*Thread handler which listens to feedback data and updates the actuator info
 * map*/
void RtInterface::parse_feedback_iface_msg_new() {

  /* Local declarations */
  std::vector<std::map<std::string,std::string>> info_key_value_map_vector;
  std::map<std::string,std::string> info_key_value_map;
  std::string axis_name;
  bool unpack_status = false;
  std::string raw_string;
  bool update_status = false;
  int axes_it;

  while (get_rt_interface_conn_status()) {

    unpack_status = false;
    retrieve_feedback_iface_msg(&raw_string);
    //logger_->debug("Raw Message: {}", raw_string);

    auto start_parsing_time = std::chrono::high_resolution_clock::now();
//    std::size_t msg_delim_pos_carr_ret = packed_msg.find('\r');

    // Deserialise rawstring into map of key value pairs using protobuf
    unpack_status = protobuf_parser_->unpack_msg(raw_string, &info_key_value_map_vector);

    //Update each axis received in the message 
    if(unpack_status) {

        // size of the vector of maps tells us number of axes in the message
        int num_axes = info_key_value_map_vector.size();

        logger_fb->debug("Number of axes in feedback message : {}",num_axes);

        //Iterate through the axes and update data map
        axes_it = 0;
        while(axes_it<num_axes)
        {
          //Get key value map for axis
          info_key_value_map = info_key_value_map_vector[axes_it];

          logger_fb->debug("FB for axis {} at ts {}",info_key_value_map["sensor_id"],info_key_value_map["ts_msec"]);

          //Get axis name from actuator map
          unpack_status = get_accessory_name_new(info_key_value_map, &axis_name);

          if (unpack_status) {
            /* Remove \n from the string*/
            //substr.pop_back();
            //substr.append("\rR:OK\n");
            update_status = update_data_map_new(axis_name, info_key_value_map);
            if(update_status) {
              feedback_data_state_map_[axis_name] = true;
            }

          } else {
              logger_fb->debug("Invalid sensor id in the proto msg!");
              //substr.pop_back();
              //substr.append("\rR:ERROR\n");
          }

          axes_it++;
        }
    }

    info_key_value_map_vector.clear();
    feedback_data_avail_ = true;
    feedback_data_cv_.notify_one();

    if (unpack_status ) {

      auto completed_parsing_time = std::chrono::high_resolution_clock::now();

      std::chrono::microseconds dt_p =
        std::chrono::duration_cast<std::chrono::microseconds> (
          completed_parsing_time - start_parsing_time);

      logger_fb->debug("parsing successful, dT_P: {} us , string buffer size: {}",
        dt_p.count(), 0);


    } else {
      logger_fb->debug("parsing unsuccessful. continue.. ");
    }
  }
}

/* Deprecated function !! */
void RtInterface::parse_feedback_iface_msg() {

  std::vector<std::string> msg_substrings;
  std::string rt_key_value_messages;
  std::string rt_key_value_string;

  bool unpack_status = false;
  std::string raw_string;
  std::string packed_msg;

  while (get_rt_interface_conn_status()) {

    unpack_status = false;
    retrieve_feedback_iface_msg(&raw_string);
    packed_msg.clear();
    packed_msg = raw_string;
    logger_->debug("Raw Message: {}", raw_string);
    raw_string.clear();

    auto start_parsing_time = std::chrono::high_resolution_clock::now();
//    std::size_t msg_delim_pos_carr_ret = packed_msg.find('\r');

    msg_substrings.clear();
    boost::split(msg_substrings, packed_msg, [](char c) { return c == '\n';});
    logger_->debug("after split :");
    for (auto &substring: msg_substrings) {
        logger_->debug("{}",substring);
    }
    packed_msg.clear();

    for (auto &substring: msg_substrings) {

        std::size_t head;
        std::string substr = substring;
        std::size_t tail = substr.find(ProtocolParser::message_delim_);
        std::string axis_feedback;

        logger_->debug("rt feedback msg substring: {}",substring);

        while (tail != std::string::npos) {

            head = substr.find(ProtocolParser::header_);
            if(std::string::npos != head) {

                if (head < tail) {
                    /*remove head and tail*/
                    axis_feedback = substr.substr(head + 1, tail - 1);

                    logger_->debug("Axis Feedback Msg: {}", axis_feedback);
                    unpack_status =
                        unpack_feedback_msg(axis_feedback);
                    substr.erase(head, tail + 2);
                } else {
                    substr.erase(0, tail + 2);
                }
            }

            else {
                substr.erase(0, tail+3);
            }

            tail = substr.find(ProtocolParser::message_delim_);
        }

    }

    for(auto &substring: msg_substrings) {
        substring.clear();
    }

    if (unpack_status ) {
      feedback_data_avail_ = true;
      feedback_data_cv_.notify_one();

      auto completed_parsing_time = std::chrono::high_resolution_clock::now();

      std::chrono::microseconds dt_p =
        std::chrono::duration_cast<std::chrono::microseconds> (
          completed_parsing_time - start_parsing_time);

      logger_->debug("parsing successful, dT_P: {} us , string buffer size: {}",
        dt_p.count(), 0);


    } else {
      logger_->debug("parsing unsuccessful. continue.. ");
    }
  }
}

void RtInterface::send_feedback_iface_msg(const std::string& str) {

    /*Push the string to txq for transmitting it to RT system*/
    logger_->debug("Message to be transmitted to feedback interface : {}", str);
//    feedback_interface_->txq->push(str.c_str());
//    feedback_interface_->push(str);
    feedback_interface_->send_message(str);
}

bool RtInterface::pack_rt_msg(const std::string &axis_name,
    std::vector<std::pair<std::string, std::string>> *key_value_vector,
    std::string *packed_string) {

    protocol_parser_->pack_msg(axis_name, key_value_vector, packed_string);
    logger_->debug("Packed Msg: {}", *packed_string);
    return true;
}

bool RtInterface::unpack_control_iface_msg(const std::string& str) {

    std::vector<std::pair<std::string, std::string>> key_value_vector;
    std::string axis_name;
    std::string response_string = str;
    bool update_status = false;

    bool unpack_status = protocol_parser_->unpack_msg(str, &axis_name,
     &key_value_vector);

    if(unpack_status) {
      update_status = update_job_status(axis_name, &key_value_vector);
    }

    if (update_status) {
        /* Remove \n from the string*/
        response_string.pop_back();
        response_string.append("\rR:OK\n");

    } else {
        response_string.pop_back();
        response_string.append("\rR:ERROR\n");
    }

    // TODO(inderjeet.y@greyorange.sg): update the params hash table and send
    // the response to rt
  return update_status;
}

bool RtInterface::pack_command_msg(const std::string &axis_name,
   const enum_command_req_msg_type command_type, std::vector<Json::Value>* command,
   std::string *msg) {
  
  bool result;
  int axis_id;
  result = get_accessory_id(axis_name,axis_id);
  //protocol_parser_->pack_cmd_msg(axis_name, command_type, command, msg);

  if(result)
  {
    protobuf_parser_->pack_cmd_msg(axis_id, command_type, command, msg);
  }

  logger_->debug("Packed cmd msg: {}", *msg);
  return result;
}

bool RtInterface::unpack_feedback_msg(std::string &msg) {

  std::vector<std::pair<std::string, std::string>> key_value_vector;
  std::string axis_name;
  int axis_id;
  bool unpack_status ;
  std::vector<std::pair<std::string, std::string>> info_key_value_vector;
  std::string response_string = msg;

  unpack_status = protocol_parser_->unpack_feedback_msg(msg, &axis_id,
    &axis_name, &info_key_value_vector);

  bool update_status = false;

  if(unpack_status) {

      unpack_status = get_accessory_name(&info_key_value_vector, &axis_name);

      if (unpack_status) {
          /* Remove \n from the string*/
          response_string.pop_back();
          response_string.append("\rR:OK\n");
          update_status = update_data_map(axis_name, &info_key_value_vector);
          if(update_status) {
            feedback_data_state_map_[axis_name] = true;
          }

      } else {
          response_string.pop_back();
          response_string.append("\rR:ERROR\n");
      }
  }

  return update_status;
}

bool RtInterface::get_accessory_id(std::string axis_name,int &axis_id)
{
  bool status = false;
  auto actuator_ptr = actuator_map_.find(axis_name);


  if(actuator_ptr == actuator_map_.end()) {

     logger_->warn("!!! Unregistered actuator name received !!!:{}",axis_name);
     return status;
   }

   else {
     status = true;
     axis_id = std::stoi(actuator_ptr->second->get_accessory_idx());
     logger_->debug("Found axis id {} for axis name {}",axis_id,axis_name);
      return status;
   }
   
}

bool RtInterface::get_accessory_name_new(
  std::map<std::string, std::string> key_value_map,
  std::string *axis_name) {

    //map sensor_id to accessory_name
    bool valid_sensor_id = false;

    for(auto it=actuator_map_.begin();it!=actuator_map_.end();it++)
    {
      if(it->second->get_accessory_idx()==key_value_map["sensor_id"])
      {
        valid_sensor_id = true;
        *axis_name = it->first;
        return valid_sensor_id;
      }

    }
    return valid_sensor_id;
  }

bool RtInterface::get_accessory_name(
  std::vector<std::pair<std::string, std::string>>* key_value_vector,
  std::string *axis_name) {

  auto itr = key_value_vector->begin();

  if (key_value_vector->size() >= 5) {
    itr++;
    itr++;
    itr++;
    itr++; // axis_idx++
    *axis_name = itr->second;
    return true;

  } else {

    return false;
  }

}


 bool RtInterface::update_feedback() {
     std::size_t actuator_index = -1;
     std::size_t index = 0;
     std::unique_lock<std::mutex> lock(feedback_data_mutex_);

     while (get_rt_interface_conn_status()) {
       logger_fb->debug("Waiting for data in update feedback thread");
       feedback_data_cv_.wait(lock, [this] {return feedback_data_avail_;});
       for (auto& actuator_map_itr:actuator_map_) {
         std::string axis_name = actuator_map_itr.first;
         auto actuator = actuator_map_itr.second;

         if(feedback_data_state_map_[axis_name]) {
           actuator->actuator_info_.update_feedback_with_map();
           feedback_data_state_map_[axis_name] = false;
         }

       }
       feedback_data_avail_ = false;
       feedback_data_cv_.notify_one();
       logger_fb->debug("Feedback data updated");
     }
 }

 bool RtInterface::update_job_status_new(std::map<std::string,std::string> info_key_value_map) {
    bool status = false;
    ActuatorControls::job_status_s job_status;
    

    logger_fb->debug("Updating job status in the corresponding actuator map!");
     for (auto itr = info_key_value_map.begin(); itr != info_key_value_map.end(); itr++) {
     std::string key = itr->first; 
     std::string value = itr->second;
  
     if("job_id" == key) {
       job_status.job_id = std::stoi(value);
     }

     else if("success" == key) {
       job_status.success = std::stoi(value);
     }

     else if("err_source" == key) {
       job_status.error = std::stoi(value);
     }

     else if("err_code" == key) {
       job_status.sub_error = std::stoi(value);
     }

     logger_fb->debug("Key:Value::{}:{}", key, value);

   }

    // Since we donot know which actuator this job_id corresponds to
    // iterator through  all actuators
    /** @TODO: Add another field for axis name in main_control_response_msg */
    for(auto actuator_ptr = actuator_map_.begin();actuator_ptr!=actuator_map_.end();actuator_ptr++)
    {
      status=actuator_ptr->second->set_job_status(job_status);
      if(status)
      {
        logger_fb->info(" job status received and updated for {}",actuator_ptr->first);
        logger_fb->info("Job Status: job_id: {}, success = {}, error_source= {}, sub_error = {}",job_status.job_id, job_status.success, job_status.error, job_status.sub_error);
        break;
      }
    }

    return status;
 }



 bool RtInterface::update_job_status(std::string axis_name, std::vector<std::pair<std::string, std::string>> *key_value_vector) {

   bool status = false;
   ActuatorControls::job_status_s job_status;
   int num_of_fields = 4;


   auto itr = key_value_vector->begin();

   // seq_id
   itr++;
   // ts_secs
   itr++;
   // ts_msecs
   itr++; //axis_idx++
   itr++;
   axis_name = itr->second;
   itr++; // axis_name++

   logger_->debug("Set Key value pairs: ");

   int i = 0;
   for (; itr != key_value_vector->end(); ++itr) {
     std::string key = itr->first;
     std::string value = itr->second;

     if("job_id" == key) {
       job_status.job_id = std::stoi(value);
       i++;
     }

     else if("success" == key) {
       job_status.success = std::stoi(value);
       i++;
     }

     else if("err_source" == key) {
       job_status.error = std::stoi(value);
       i++;
     }

     else if("err_code" == key) {
       job_status.sub_error = std::stoi(value);
       i++;
     }

     logger_->debug("{}Key:Value::{}:{}",i, key, value);

   }

   auto actuator_ptr = actuator_map_.find(axis_name);

   if(actuator_ptr == actuator_map_.end()) {

     logger_->warn("!!! Unregistered actuator job status received !!!:{}",axis_name);
     return status;
   }

   else {


     if(i == num_of_fields) {

       job_status.is_recvd = true;
       actuator_map_.at(axis_name)->set_job_status(job_status);
       logger_->info(" job status received and updated for {}",axis_name);
       logger_->info("Job Status: job_id: {}, success = {}, error_source= {}, sub_error = {}",job_status.job_id, job_status.success, job_status.error, job_status.sub_error);

     }

   }


}

 bool RtInterface::update_data_map_new(const std::string axis_name,
  std::map<std::string, std::string> key_value_map) {


  auto actuator_ptr = actuator_map_.find(axis_name);
  bool status = false;

  if(actuator_ptr == actuator_map_.end()) {

    logger_->warn("!!! Unregistered actuator feedback received !!!:{}",axis_name);
    return status;

  }

  else {

    logger_fb->debug("Setting Key value pairs for actuator {} ",axis_name);

    int i=0;
    for(auto it = key_value_map.begin();it!=key_value_map.end();it++)
    {
      actuator_map_.at(axis_name)->set_actuator_info(it->first,it->second);
      logger_fb->debug("{}Key:Value::{}:{}",i, it->first, it->second);
      i++;
    }

    int num_fields_feedback = 5;
    if ( num_fields_feedback < i)
    {
          status = true;
    }

    else
    {
          logger_fb->warn("!!! Incomplete feedback received !!!");
          status =  false;
    }
  }

  return status;


  }

 bool RtInterface::update_data_map(const std::string axis_name,
  std::vector<std::pair<std::string, std::string>> *key_value_vector) {

  auto actuator_ptr = actuator_map_.find(axis_name);
  bool status = false;

  if(actuator_ptr == actuator_map_.end()) {

    logger_->warn("!!! Unregistered actuator feedback received !!!:{}",axis_name);
    return status;

  }

  else {

    auto itr = key_value_vector->begin();
    actuator_map_.at(axis_name)->set_actuator_info(itr->first,itr->second);
    // seq_id

    itr++;
    actuator_map_.at(axis_name)->set_actuator_info(itr->first,itr->second);
    // ts_secs

    itr++;
    actuator_map_.at(axis_name)->set_actuator_info(itr->first,itr->second);
    // ts_msecs

    itr++; //axis_idx++
    itr++; // axis_name++
    itr++; //enc_handle

    logger_->debug("Set Key value pairs: ");
    int i = 0;
    for (; itr != key_value_vector->end(); ++itr) {
      std::string key = itr->first;
      std::string value = itr->second;
      actuator_map_.at(axis_name)->set_actuator_info(key, value);
      logger_->debug("{}Key:Value::{}:{}",i, key, value);
      i++;
    }

    int num_fields_feedback = 4;
    if ( num_fields_feedback <= i)
    {
          status = true;
    }

    else
    {
          status =  false;
    }
  }

  return status;

}

void RtInterface::send_actuators_params_to_rt(const std::string axis_name,
  std::vector<std::pair<std::string, std::string>> *params_vector) {

  logger_->debug("sending: {}  actuator params to  rt ", axis_name);
  logger_->debug("params_vector: ");

  for (auto & param_itr : *params_vector) {

    logger_->debug("{} : {}", param_itr.first,
      param_itr.second);
  }

  std::string rt_message;
  bool pack_status = pack_rt_msg(axis_name, params_vector, &rt_message);

  if (pack_status) {
    send_control_iface_msg(rt_message);
  } else {
    logger_->debug("Error in packing msg");
  }
}

void RtInterface::send_actuators_params_to_rt_class_wise(void) {
    // std::size_t index = 0;
    // std::cout<< "sending actuator params data to  rt" << std::endl;
    // // std::cout<< "no of actuators: "<< actuators_ptr_list_.size()<< std::endl;
    // // for (int actuator_itr = 0; actuator_itr < actuators_ptr_list_.size();
    // //     actuator_itr++) {
    // //     std::vector<std::pair<std::string, std::string>> params_vector;
    // //     // actuators_ptr_list_.at(
    // //         // index)->get_accessory_params_class_wise(&params_vector);

    // //     std::cout<< "params_vector class wise:"<< std::endl;
    // //     for (int param_itr = 0; param_itr < params_vector.size(); ++param_itr) {
    // //         std::cout << params_vector[param_itr].first
    // //          << ":" << params_vector[param_itr].second<< std::endl;
    // //     }

    // std::string rt_message = pack_rt_msg(
    //     actuators_ptr_list_.at(index)->get_accessory_name(),
    //     &params_vector);
    // send_control_iface_msg(rt_message);
    // ++index;
}



void RtInterface::send_actuators_controls_to_rt() {

    // for (int actuator_itr = 0; actuator_itr < actuators_ptr_list_.size();
    // actuator_itr++) {
    //     std::vector<std::pair<std::string, std::string>> controls_vector;
    //     // actuators_ptr_list_.at(actuator_itr)->get_accessory_controls
    //     // (&controls_vector);

    //     for (int controls_itr = 0; controls_itr < controls_vector.size();
    //      ++controls_itr) {
    //         std::cout << "controls_vector :" <<
    //          controls_vector[controls_itr].first << "," <<
    //           controls_vector[controls_itr].second<< std::endl;
    //     }

    //     std::string rt_message = pack_rt_msg(
    //         actuators_ptr_list_.at(actuator_itr)->get_accessory_name(),
    //         &controls_vector);
    //     send_control_iface_msg(rt_message);
    // }
}

void RtInterface::send_actuators_info_params_to_rt() {

    // for (int actuator_itr = 0; actuator_itr < actuators_ptr_list_.size();
    // actuator_itr++) {
    //     std::vector<std::pair<std::string, std::string>> info_vector;
    //     actuators_ptr_list_.at(actuator_itr)->get_accessory_controls(&info_vector);
    //     std::string rt_message = pack_rt_msg(
    //         actuators_ptr_list_.at(actuator_itr)->get_accessory_name(),
    //         &info_vector);
    //     send_feedback_iface_msg(rt_message);
    // }
}


// void RtInterface::send_actuators_info_params_to_rt_class_wise(void) {


// }