/**
 * \file tcp_connection.cpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief
 * @version 0.1
 * \date 16-07-2019
 *
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 *
 */

#include <boost/algorithm/string.hpp>
#include <ctime>
#include <string>
#include "tcp_connection.hpp"



using boost::asio::ip::tcp;

/**
 * \brief Construct a new Tcp Connection:: Tcp Connection object
 *
 * \param io_service
 */

std::condition_variable TcpConnection::dataCondition;

TcpConnection::TcpConnection(boost::asio::io_service& io_service, uint32_t port,
                MessageQueue<std::string>* rxq_server, MessageQueue<std::string>* txq_server,std::string on_connect_message):
    connection_status_(false),
    port_(port),
    socket_(io_service),
    txq(txq_server),
    rxq(rxq_server),
    on_connect_message(on_connect_message)
{
  
  logger_ = spdlog::get("butler_control_interface")->clone("tcp_connection");

  logger_->debug(
      "Created a TCP Connection Object and starting to listen"
      " on port: {}",
      port_);

}
TcpConnection::~TcpConnection()
{
  /* Just for precautions */
  close_socket();
  send_message_thread_.join();
}



tcp::socket& TcpConnection::socket()
{
  return socket_; 
}

void TcpConnection::close_socket()
{ 
  connection_status_ = false;
  TcpConnection::dataCondition.notify_one();
  socket_.close();
}

void TcpConnection::start_write() {

  try
  {
    std::string buffer_send;
    buffer_send = txq->pop();
    logger_->debug("start_write data: {}", buffer_send);
    boost::asio::async_write(
        socket_, boost::asio::buffer(buffer_send),
        boost::bind(&TcpConnection::handle_write, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }
  catch (std::exception& e)
  {
    std::cout << "Port " << this->port_ << " Exception Occured in tcp connection start write " << e.what() << std::endl;
  }
}

void TcpConnection::handle_write(const boost::system::error_code& error,
                                 size_t bytes_transferred) {
  if (!error) {
    logger_->debug("Written {} Bytes",bytes_transferred);
  }

  else {
    logger_->debug("handle write error:{}", error.message());
  }
}

void TcpConnection::start_read() {
  socket_.async_receive(
      boost::asio::buffer(receive_buffer, TCP_BUFFER_SIZE), 0,
      boost::bind(&TcpConnection::handle_read, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void TcpConnection::handle_read(const boost::system::error_code& error,
                                size_t bytes_transferred)
{
  if (!error)
  {
    std::string message;
    message.append(receive_buffer, bytes_transferred);
    //TODO: add try catch for raw msg
    try {
        logger_->debug("Received {}", message);
        this->rxq->push(message);
    }
    catch(boost::exception &ec) {
      logger_->debug("Caught exception on raw message logging ");
      std::cout << "Raw Message Exception " << message << std::endl;
    }
    start_read();
  }

  else {
    logger_->error("Error on Receive, Error: {}",error.message());
    close_socket();
  }
}

bool TcpConnection::send_thread_handler() {
  while (connection_status_){
    std::string buffer_send;
    logger_->debug("Waiting for data to arrive in send buffer\n");
    wait_on_queue(txq);

    if(!connection_status_) {
      break;
    }
    start_write();
  }
}


 
void TcpConnection::retreive_message(std::string* msg)
{ 
  rxq->wait();
  *msg = rxq->pop();
}


bool TcpConnection::get_conn_status()
{
  return connection_status_;
}



void TcpConnection::start()
{
  rxq->resetBreakWait();
  txq->resetBreakWait();
  connection_status_ = true;
  conn_status_cv_.notify_one();
  send_message_thread_ = std::thread(&TcpConnection::send_thread_handler, this);
  logger_->debug("Starting Client : {}", name_);

  boost::asio::ip::tcp::no_delay option(true);
  boost::system::error_code ec;
  socket_.set_option(option, ec);
  if (ec)
  {
    logger_->debug("asio set option err: {}",ec.message());
  }


  std::string epoch_string;
  epoch_string = "{time/epoch=";
  std::time_t epoch = std::time(nullptr);
  epoch_string += std::to_string(epoch);
  epoch_string += "&}";
  send_message(epoch_string);

  std::vector<std::string> substrings;
  boost::split(substrings, on_connect_message, [](char c){ return c == '}';});

  for (auto & str: substrings) {
      str.append("}");
      logger_->debug("On connect message: {}", str);
      send_message(str);
  }
  start_read();
}

bool TcpConnection::predicate_send()
{
  /*
  while (!pred()) {
    wait(lock);
}*/
  bool status_q = txq->empty();
  bool rv = !connection_status_ || !status_q;
  return rv;
}

void TcpConnection::wait_on_queue(MessageQueue<std::string>* queue) {
    std::unique_lock<std::mutex> lock(tx_mutexLock);
    logger_->debug("waiting for predicate");
    TcpConnection::dataCondition.wait(lock, [this] {return predicate_send();});
    logger_->debug("wait on queue exit:this->txq->empty() = {},connection_status_ = {}", queue->empty(), connection_status_);
}

void TcpConnection::send_message(const std::string &msg) {
  txq->push(msg);
  TcpConnection::dataCondition.notify_all();
}