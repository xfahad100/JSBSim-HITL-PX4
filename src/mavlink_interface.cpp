/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2020 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mavlink_interface.h"

MavlinkInterface::MavlinkInterface()
    : received_first_actuator_(false),
      gcs_socket_fd_(0),
      sdk_socket_fd_(0),
      simulator_socket_fd_(0),
      simulator_tcp_client_fd_(0),
      serial_enabled_(true),
      hil_mode_(false),
      hil_state_level_(false),
      enable_lockstep_(true),
      use_tcp_(false),
      m_status{},
      m_buffer{},
      mavlink_addr_str_("INADDR_ANY"),
      mavlink_udp_port_(kDefaultMavlinkUdpPort),
      //mavlink_tcp_port_(kDefaultMavlinkTcpPort),
      gcs_udp_port_(kDefaultGCSUdpPort),
      sdk_udp_port_(kDefaultSDKUdpPort),
      gcs_addr_("INADDR_ANY"),
      sdk_addr_("INADDR_ANY"),
      io_service(),
      serial_dev(io_service),
      device_(kDefaultDevice),
      baudrate_(kDefaultBaudRate),
      tx_q{},
      rx_buf{},
      imu_updated_(false),
      baro_updated_(false),
      diff_press_updated_(false),
      mag_updated_(false) {}

MavlinkInterface::~MavlinkInterface() { close(); }

void MavlinkInterface::Load() {
  mavlink_addr_ = htonl(INADDR_ANY);
  if (mavlink_addr_str_ != "INADDR_ANY") {
    mavlink_addr_ = inet_addr(mavlink_addr_str_.c_str());
    if (mavlink_addr_ == INADDR_NONE) {
      std::cerr << "Invalid mavlink_addr: " << mavlink_addr_str_ << ", aborting\n";
      abort();
    }
  }
  local_gcs_addr_.sin_port = 0;
  if (gcs_addr_ != "INADDR_ANY") {
    local_gcs_addr_.sin_port = inet_addr(gcs_addr_.c_str());
    if (local_gcs_addr_.sin_port == 0) {
      std::cerr << "Invalid gcs_addr: " << gcs_addr_ << ", aborting\n";
      abort();
    }
  }
  if (sdk_addr_ != "INADDR_ANY") {
    local_sdk_addr_.sin_port = inet_addr(sdk_addr_.c_str());
    if (local_sdk_addr_.sin_port == 0) {
      std::cerr << "Invalid sdk_addr: " << sdk_addr_ << ", aborting\n";
      abort();
    }
  }

  if (hil_mode_) {
    local_gcs_addr_.sin_family = AF_INET;
    local_gcs_addr_.sin_port = htons(0);
    local_gcs_addr_len_ = sizeof(local_gcs_addr_);

    remote_gcs_addr_.sin_family = AF_INET;
    remote_gcs_addr_.sin_port = htons(gcs_udp_port_);
    remote_gcs_addr_len_ = sizeof(remote_gcs_addr_);

    local_sdk_addr_.sin_family = AF_INET;
    local_sdk_addr_.sin_port = htons(0);
    local_sdk_addr_len_ = sizeof(local_sdk_addr_);

    remote_sdk_addr_.sin_family = AF_INET;
    remote_sdk_addr_.sin_port = htons(sdk_udp_port_);
    remote_sdk_addr_len_ = sizeof(remote_sdk_addr_);

    if ((gcs_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      std::cerr << "Creating GCS UDP socket failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if (bind(gcs_socket_fd_, (struct sockaddr *)&local_gcs_addr_, local_gcs_addr_len_) < 0) {
      std::cerr << "GCS UDP bind failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if ((sdk_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      std::cerr << "Creating SDK UDP socket failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if (bind(sdk_socket_fd_, (struct sockaddr *)&local_sdk_addr_, local_sdk_addr_len_) < 0) {
      std::cerr << "SDK UDP bind failed: " << strerror(errno) << ", aborting\n";
      abort();
    }
  }

  if (serial_enabled_) {
    // Set up serial interface
    io_service.post(std::bind(&MavlinkInterface::do_read, this));

    // run io_service for async io
    io_thread = std::thread([this]() { io_service.run(); });
    open();

  } else {
    memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
    remote_simulator_addr_.sin_family = AF_INET;
    remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

    memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
    local_simulator_addr_.sin_family = AF_INET;
    local_simulator_addr_len_ = sizeof(local_simulator_addr_);

    if (use_tcp_) {
        std::cerr << "still no " << std::endl; //check

      local_simulator_addr_.sin_addr.s_addr = htonl(mavlink_addr_);
      local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Creating TCP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      int yes = 1;
      int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      struct linger nolinger {};
      nolinger.l_onoff = 1;
      nolinger.l_linger = 0;

      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // The socket reuse is necessary for reconnecting to the same address
      // if the socket does not close but gets stuck in TIME_WAIT. This can happen
      // if the server is suddenly closed
      int socket_reuse = 1;
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // Same as above but for a given port
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // set socket to non-blocking
      result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
      if (result == -1) {
        std::cerr << "setting socket to non-blocking failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        std::cerr << "bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      errno = 0;
      if (listen(simulator_socket_fd_, 0) < 0) {
        std::cerr << "listen failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      memset(fds_, 0, sizeof(fds_));
      fds_[LISTEN_FD].fd = simulator_socket_fd_;
      fds_[LISTEN_FD].events = POLLIN;  // only listens for new connections on tcp

    } else {
      remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
      remote_simulator_addr_.sin_port = htons(mavlink_udp_port_);

      local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
      local_simulator_addr_.sin_port = htons(0);

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Creating UDP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // set socket to non-blocking
      int result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
      if (result == -1) {
        std::cerr << "setting socket to non-blocking failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        std::cerr << "bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      memset(fds_, 0, sizeof(fds_));
      fds_[CONNECTION_FD].fd = simulator_socket_fd_;
      fds_[CONNECTION_FD].events = POLLIN | POLLOUT;  // read/write
    }
  }
}

void MavlinkInterface::SendActuatorMsgs(Eigen::Vector4d out_raw) {



// Pack the message
//   mavlink_heartbeat_t heart;
//    mavlink_message_t msght;
//   heart.type=MAV_ODID_UA_TYPE_AEROPLANE;
//   heart.autopilot=MAV_AUTOPILOT_GENERIC;

// mavlink_msg_heartbeat_encode(1,0,&msght,&heart);
//  send_mavlink_message(&msght);



                   // mavlink_servo_output_raw_t serm2;
                    //mavlink_msg_servo_output_raw_decode(&massg, &serm2);
//                   usleep(25000);
                    //std::cout << "servo1::  " << serm2.servo1_raw <<endl;

//                    mavlink_msg_scaled_imu_decode(&massg , &imu1);
//                   std::cout << "big::  " << imu1.zacc <<endl;
  //              }
//}
//usleep(5000);
//std::cout << "elev::  " << out_b_[0]<<endl;
//std::cout << "ail::  " << out_b_[2]<<endl;
//std::cout << "rud::  " << out_b_[2]<<endl;




  elevatorDefDeg = out_raw[0] * 57.3;
  aileronDefDeg  = out_raw[1] * 57.3;
  rudderDefDeg   = out_raw[2] * 57.3;
  throttleDef    = out_raw[3];
  elevatorDefPWM =  mapOneRangeToAnother(elevatorDefDeg,-15,15,1000,2000,2);
  aileronDefPWM  =  mapOneRangeToAnother(aileronDefDeg,-20,20,1000,2000,2);
  rudderDefPWM  =   mapOneRangeToAnother(rudderDefDeg, -20, 20, 1000, 2000, 2);
  throttleDefPWM  =   mapOneRangeToAnother(throttleDef, 0.0, 1.0, 1000, 2000, 2);

  std::cout << "pwm::  " << throttleDef<<endl;
  std::cout << "ailerpwm::  " << aileronDefPWM<<endl;

mavlink_rc_channels_override_t rc_override; //Msg to override rc channels
mavlink_message_t msg_rc;
rc_override.chan1_raw=aileronDefPWM;
rc_override.chan2_raw=elevatorDefPWM;  //servo port2
rc_override.chan3_raw=throttleDefPWM;  //throttle
rc_override.chan4_raw=rudderDefPWM;  //servo port1
rc_override.chan5_raw=UINT16_MAX;
rc_override.chan6_raw=UINT16_MAX;
rc_override.chan7_raw=UINT16_MAX;
rc_override.chan8_raw=UINT16_MAX;
rc_override.target_system = 1; // Send command to MAV 001
rc_override.target_component = MAV_COMP_ID_AUTOPILOT1;//PX_COMP_ID_ALL;
mavlink_msg_rc_channels_override_encode(255, 0, &msg_rc, &rc_override);
send_mavlink_message(&msg_rc);
//usleep(500);


//std::cout << " servo 1::  " << servo.servo1_raw <<endl;


}

void MavlinkInterface::SendGpsMessages(const SensorData::Gps &data) {


}

void MavlinkInterface::UpdateBarometer(const SensorData::Barometer &data) {
  temperature_ = data.temperature;
  abs_pressure_ = data.abs_pressure;
  pressure_alt_ = data.pressure_alt;

  baro_updated_ = true;
}

void MavlinkInterface::UpdateAirspeed(const SensorData::Airspeed &data) {
  diff_pressure_ = data.diff_pressure;

  diff_press_updated_ = true;
}

void MavlinkInterface::UpdateIMU(const SensorData::Imu &data) {
  accel_b_ = data.accel_b;
  gyro_b_ = data.gyro_b;
  imu_updated_ = true;
}

void MavlinkInterface::UpdateMag(const SensorData::Magnetometer &data) {
  mag_b_ = data.mag_b;

  mag_updated_ = true;
}

void MavlinkInterface::pollForMAVLinkMessages() {
  if (gotSigInt_) {
    return;
  }

  bool received_actuator = false;

  do {
    int timeout_ms = (received_first_actuator_ && enable_lockstep_) ? 1000 : 0;
    int ret = ::poll(&fds_[0], N_FDS, timeout_ms);

    if (ret < 0) {
      std::cerr << "poll error: " << strerror(errno) << "\n";
      return;
    }

    if (ret == 0 && timeout_ms > 0) {
      std::cerr << "poll timeout\n";
      return;
    }

    for (int i = 0; i < N_FDS; i++) {
      if (fds_[i].revents == 0) {
        continue;
      }

      if (!(fds_[i].revents & POLLIN)) {
        continue;
      }

      if (i == LISTEN_FD) {  // if event is raised on the listening socket
        acceptConnections();
      } else {  // if event is raised on connection socket
        int ret = recvfrom(fds_[i].fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_simulator_addr_,
                           &remote_simulator_addr_len_);
        if (ret < 0) {
          // all data is read if EWOULDBLOCK is raised
          if (errno != EWOULDBLOCK) {  // disconnected from client
            std::cerr << "recvfrom error: " << strerror(errno) << "\n";
          }
          continue;
        }

        // client closed the connection orderly, only makes sense on tcp
        if (use_tcp_ && ret == 0) {
          std::cerr << "Connection closed by client."
                    << "\n";
          close_conn_ = true;
          continue;
        }

        // data received
        int len = ret;
        mavlink_message_t msg;
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i) {
          if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status)) {
             // std::cout << "xacc::  " << _buf[i] <<endl;
            if (hil_mode_) {
              send_mavlink_message(&msg);
            }
            handle_message(&msg, received_actuator);
          }
        }
      }
    }
  } while (!close_conn_ && received_first_actuator_ && !received_actuator && enable_lockstep_ && !gotSigInt_);
}

void MavlinkInterface::pollFromGcsAndSdk() {
    //std::cerr << "im here haha "<< "\n";

  struct pollfd fds[2] = {};
  fds[0].fd = gcs_socket_fd_;
  fds[0].events = POLLIN;
  fds[1].fd = sdk_socket_fd_;
  fds[1].events = POLLIN;

  const int timeout_ms = 0;

  int ret = ::poll(&fds[0], 2, timeout_ms);

  if (ret < 0) {
    std::cerr << "poll error: " << strerror(errno) << "\n";
    return;
  }

  if (fds[0].revents & POLLIN) {
    int len =
        recvfrom(gcs_socket_fd_, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_gcs_addr_, &remote_gcs_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, _buf[i], &msg, &status)) {
          // forward message from GCS to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }

  if (fds[1].revents & POLLIN) {
    int len =
        recvfrom(sdk_socket_fd_, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_sdk_addr_, &remote_sdk_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_2, _buf[i], &msg, &status)) {
          // forward message from SDK to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }
}

void MavlinkInterface::acceptConnections() {
  if (fds_[CONNECTION_FD].fd > 0) {
    return;
  }

  // accepting incoming connections on listen fd
  int ret = accept(fds_[LISTEN_FD].fd, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

  if (ret < 0) {
    if (errno != EWOULDBLOCK) {
      std::cerr << "accept error: " << strerror(errno) << "\n";
    }
    return;
  }

  // assign socket to connection descriptor on success
  fds_[CONNECTION_FD].fd = ret;                   // socket is replaced with latest connection
  fds_[CONNECTION_FD].events = POLLIN | POLLOUT;  // read/write
}

void MavlinkInterface::handle_message(mavlink_message_t *msg, bool &received_actuator) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
      const std::lock_guard<std::mutex> lock(actuator_mutex);

      mavlink_hil_actuator_controls_t controls; ///  SET the command to connect HIL actuator control from MAVLink to JSB
      mavlink_msg_hil_actuator_controls_decode(msg, &controls);   /// Decode the control input msg from MAVLink to JSB
     // armed_ = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);       /// Safety Arming before data communication
      

      for (unsigned i = 0; i < n_out_max; i++) {
        input_index_[i] = i;
      }

      // set rotor speeds, controller targets
//      input_reference_.resize(n_out_max);                       /// Controller data reference
//      for (int i = 0; i < input_reference_.size(); i++) {       /// loop for data fetching in an array type where i is an internal variable...
//        input_reference_[i] = controls.controls[i];
//        //std::cout << "input:  " << input_reference_[i]  <<endl;
//      }

      received_actuator = true;
      received_first_actuator_ = true;
      break;

  }
}


void MavlinkInterface::forward_mavlink_message(const mavlink_message_t *message) {
  if (gotSigInt_) {
    return;
  }

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);
  ssize_t len;
  if (gcs_socket_fd_ > 0) {
    len = sendto(gcs_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_gcs_addr_, remote_gcs_addr_len_);

    if (len <= 0) {
      std::cerr << "Failed sending mavlink message to GCS: " << strerror(errno) << "\n";
    }
  }

  if (sdk_socket_fd_ > 0) {
    len = sendto(sdk_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_sdk_addr_, remote_sdk_addr_len_);
    if (len <= 0) {
      std::cerr << "Failed sending mavlink message to SDK: " << strerror(errno) << "\n";
    }
  }
}

void MavlinkInterface::handleHighLatency(mavlink_message_t& message)
{
    if (tx_q.size() >= MAX_TXQ_SIZE) {
    mavlink_high_latency_t highLatency;
    mavlink_msg_high_latency_decode(&message, &highLatency);
    }
}

void MavlinkInterface::send_mavlink_message(const mavlink_message_t *message) {
  assert(message != nullptr);

  if (gotSigInt_ || close_conn_) {
    return;
  }

  if (serial_enabled_) {
    if (!is_open()) {
      std::cerr << "Serial port closed! \n";
      return;
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex);

      if (tx_q.size() >= MAX_TXQ_SIZE) {
        std::cout << "Tx queue overflow\n";
      }
      tx_q.emplace_back(message);
    }
    io_service.post(std::bind(&MavlinkInterface::do_write, this, true));

  } else {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);

    if (fds_[CONNECTION_FD].fd > 0) {
      int timeout_ms = (received_first_actuator_ && enable_lockstep_) ? 1000 : 0;
      int ret = ::poll(&fds_[0], N_FDS, timeout_ms);

      if (ret < 0) {
        std::cerr << "poll error: " << strerror(errno) << "\n";
        return;
      }

      if (ret == 0 && timeout_ms > 0) {
        std::cerr << "poll timeout\n";
        return;
      }

      if (!(fds_[CONNECTION_FD].revents & POLLOUT)) {
        std::cerr << "invalid events at fd:" << fds_[CONNECTION_FD].revents << "\n";
        return;
      }

      size_t len;
      if (use_tcp_) {
        len = send(fds_[CONNECTION_FD].fd, buffer, packetlen, 0);
      } else {
        len = sendto(fds_[CONNECTION_FD].fd, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_,
                     remote_simulator_addr_len_);
      }
      if (len < 0) {
        std::cerr << "Failed sending mavlink message: " << strerror(errno) << "\n";
        if (errno == ECONNRESET || errno == EPIPE) {
          if (use_tcp_) {  // udp socket remains alive
            std::cerr << "Closing connection."
                      << "\n";
            close_conn_ = true;
          }
        }
      }
    }
  }
}

void MavlinkInterface::open() {
  try {
    serial_dev.open(device_);
    serial_dev.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_dev.set_option(boost::asio::serial_port_base::character_size(8));
    serial_dev.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_dev.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_dev.set_option(
        boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    std::cout << "Opened serial device " << device_ << "\n";
  } catch (boost::system::system_error &err) {
    std::cerr << "Error opening serial device: " << err.what() << "\n";
  }
}

void MavlinkInterface::close() {
  if (serial_enabled_) {
    ::close(gcs_socket_fd_);
    ::close(sdk_socket_fd_);

    std::lock_guard<std::recursive_mutex> lock(mutex);
    if (!is_open()) return;

    io_service.stop();
    serial_dev.close();

    if (io_thread.joinable()) io_thread.join();

  } else {
    ::close(fds_[CONNECTION_FD].fd);
    fds_[CONNECTION_FD] = {0, 0, 0};
    fds_[CONNECTION_FD].fd = -1;

    received_first_actuator_ = false;
  }
}

void MavlinkInterface::do_write(bool check_tx_state) {
  if (check_tx_state && tx_in_progress) return;

  std::lock_guard<std::recursive_mutex> lock(mutex);
  if (tx_q.empty()) return;

  tx_in_progress = true;
  auto &buf_ref = tx_q.front();

  serial_dev.async_write_some(boost::asio::buffer(buf_ref.dpos(), buf_ref.nbytes()),
                              [this, &buf_ref](boost::system::error_code error, size_t bytes_transferred) {
                                assert(bytes_transferred <= buf_ref.len);
                                if (error) {
                                  std::cerr << "Serial error: " << error.message() << "\n";
                                  return;
                                }

                                std::lock_guard<std::recursive_mutex> lock(mutex);

                                if (tx_q.empty()) {
                                  tx_in_progress = false;
                                  return;
                                }

                                buf_ref.pos += bytes_transferred;
                                if (buf_ref.nbytes() == 0) {
                                  tx_q.pop_front();
                                }

                                if (!tx_q.empty()) {
                                  do_write(false);
                                } else {
                                  tx_in_progress = false;
                                }
                              });
}

void MavlinkInterface::do_read(void) {
  serial_dev.async_read_some(boost::asio::buffer(rx_buf),
                             boost::bind(&MavlinkInterface::parse_buffer, this, boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

// Based on MAVConnInterface::parse_buffer in MAVROS
void MavlinkInterface::parse_buffer(const boost::system::error_code &err, std::size_t bytes_t) {

    mavlink_message_t massg;
    mavlink_status_t _status;

    //uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    // check message is write length
   // unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &massg);


    for (auto i = 0; i < bytes_t ; i++) {
        // std::cout << "buf::  " << unsigned( _status.packet_rx_drop_count) <<endl;
        uint8_t c = rx_buf[i];
                    if (mavlink_parse_char(MAVLINK_COMM_3, c, &massg, &_status)) {

                      //  mavlink_scaled_imu_t imu1;
                        //mavlink_message_t imurev;
      //std::lock_guard<std::recursive_mutex> lock(mutex);
    //                    mavlink_message_t msgpx;
                        //mavlink_heartbeat_t packet1;
                       // mavlink_msg_heartbeat_decode(&massg, &packet1);
                       // std::cout << "msgid::  " << unsigned(massg.msgid) <<endl;  // Pixhawk sysid
    //                    std::cout << "comp::  " << unsigned(massg.compid) <<endl;


    //usleep(20000);
    switch(massg.msgid){
      case MAVLINK_MSG_ID_HIGHRES_IMU:
    {
    mavlink_highres_imu_t imuhighres;
    mavlink_msg_highres_imu_decode(&massg,&imuhighres);
    imuacc[0] = imuhighres.xacc;
    imuacc[1] = imuhighres.yacc;
    imuacc[2] = imuhighres.zacc;
   // std::cout << "imu::  " << imuacc[0] <<endl;
    break;
    }
      case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_attitude_t at1;
    mavlink_msg_attitude_decode(&massg,&at1);
      input_reference_2[0]= at1.pitch;  //degree
      input_reference_2[1]= at1.roll * 57.3;
      input_reference_2[2]= at1.yaw * 57.3;
      current = at1.time_boot_ms;
   //   gps_input[0] = (1/float((current - last)))*1000;
      //std::cout << "hertz::  " << (1/float((current - last)))*1000 <<endl;
  // std::cout << "pitchmav::  " << float(input_reference_2[0]) <<endl; //radians
//      std::cout << "roll::  " << float(input_reference_2[1]) <<endl;
//      std::cout << "yaw::  " << float(input_reference_2[2]) <<endl;
        break;
      }
      case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_heartbeat_t msghb;
        mavlink_msg_heartbeat_decode(&massg,&msghb);
        bool armed_ = (msghb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
    //std::cout << "armstate::  " << msghb.base_mode <<endl;
        break;
    }
      case MAVLINK_MSG_ID_BATTERY_STATUS:
    {
        mavlink_battery_status_t battery;
        mavlink_msg_battery_status_decode(&massg,&battery);
        //gps_input[0]= double(battery.battery_remaining);
       // cout << "batteryvolt0::  " << double((unsigned(battery.voltages[0]))/1000) <<endl;
       // cout << "batteryvolt1::  " << double((unsigned(battery.voltages[1]))/1000) <<endl;
        break;
    }
      case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
    mavlink_gps_raw_int_t gpsraw;
    mavlink_msg_gps_raw_int_decode(&massg,&gpsraw);
  //  cout << "gpsvisible::  " << unsigned(gpsraw.satellites_visible) <<endl;
    //gps_input[0]=gpsraw.lat / (double)1E7;
    //gps_input[1]=gpsraw.lon /(double)1E7;
    //gps_input[2]=gpsraw.alt /(double)1E7;
    break;
    }
      case MAVLINK_MSG_ID_VFR_HUD:
    {
     mavlink_vfr_hud_t Airpressure;
     mavlink_msg_vfr_hud_decode(&massg,&Airpressure);
     airspeed[0] = Airpressure.airspeed;
    airspeed[1] = Airpressure.climb;
    // airspeed[2] = servo.controls[2];
    std::cout << " servo 1::  " << airspeed[0] <<endl;
     std::cout << " servo 2::  " << airspeed[1] <<endl;
//     std::cout << " servo 3::  " << airspeed[2] <<endl;
//     std::cout << " servo 4::  " << servo.controls[3] <<endl;
//    std::cout << " servo 5::  " << servo.controls[4] <<endl;
     break;
    }
      case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&massg, &ack);

        if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
            if (ack.result == MAV_RESULT_ACCEPTED) {
               armack = 1;
            }
        }
        airspeed[2] = armack = 1;
       // std::cout << " armstate::  " << armack <<endl;
        break;
    }
      case MAVLINK_MSG_ID_ODOMETRY:
    {
        mavlink_odometry_t odometry;
        mavlink_msg_odometry_decode(&massg,&odometry);
        VelY= odometry.vy;
        VelZ= odometry.vz;
        VelX= odometry.vx;
        AoA = atan(VelZ/VelY);
        gamma = input_reference_2[0] - AoA;
        vUVWmag = sqrt(pow(VelX,2)+pow(VelY,2)+pow(VelZ,2));
        hdot = -vUVWmag*sin(gamma);
        gps_input[0] = hdot;
        std::cout << " hdot::  " << hdot <<endl;
        break;
    }

    }
    last = current;
                    }
    }
//  mavlink_status_t status;
//  mavlink_message_t message;
//  uint8_t *buf = this->rx_buf.data();
//  assert(rx_buf.size() >= bytes_t);

//  for (; bytes_t > 0; bytes_t--) {
//    auto c = *buf++;

//    auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
//    if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
//      _mav_parse_error(&m_status);
//      m_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
//      m_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
//      if (c == MAVLINK_STX) {
//        m_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
//        m_buffer.len = 0;
//        mavlink_start_checksum(&m_buffer);
//      }
//    }

//    if (msg_received != Framing::incomplete) {
//      if (hil_mode_) {
//        forward_mavlink_message(&message);
//      }
     // bool not_used;
      //handle_message(&message, not_used);
   // }
  //}
  do_read();
}

void MavlinkInterface::onSigInt() {
  gotSigInt_ = true;
  close();
}

Eigen::Vector3d MavlinkInterface::Getgps() {
  return gps_input;
}

Eigen::Vector3d MavlinkInterface::Getairspeed() {
  return airspeed;
}

Eigen::Vector3d MavlinkInterface::GetIMU() {
  return input_reference_2;
}

Eigen::Vector3d MavlinkInterface::imuaccel() {
  return imuacc;
}

void MavlinkInterface::SendArmedState(bool armState)
{
    static bool lastArmState = false;   // remembers previous state

    // Do nothing if state hasn't changed
    if (armState == lastArmState) {
        return;
    }

    lastArmState = armState;  // update state

    mavlink_command_long_t com = {};
    com.target_system    = 1;
    com.target_component = 1;
    com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation     = 1;
    com.param1           = armState ? 1.0f : 0.0f;   // arm / disarm
    com.param2           = 21196;                   // force arm (PX4)

    std::cout << "ARM STATE CHANGED -> "
              << (armState ? "ARM" : "DISARM") << std::endl;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(1, 0, &message, &com);
    send_mavlink_message(&message);
}

  void MavlinkInterface::SendHeartbeat()
  {
    mavlink_heartbeat_t hb;
    mavlink_message_t msg;

    hb.type = MAV_TYPE_GCS;
    hb.autopilot = MAV_AUTOPILOT_INVALID;
    hb.base_mode = 0;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(1, 0, &msg, &hb);
    send_mavlink_message(&msg);
  }

