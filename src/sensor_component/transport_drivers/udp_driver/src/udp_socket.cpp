// Copyright 2021 LeoDrive.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Developed by LeoDrive, 2021

#include "boost_udp_driver/udp_socket.hpp"

#include <rclcpp/logging.hpp>

#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace drivers
{
namespace udp_driver
{

UdpSocket::UdpSocket(
  const drivers::common::IoContext & ctx,
  const std::string & remote_ip,
  const uint16_t remote_port,
  const std::string & host_ip,
  const uint16_t host_port, 
  const size_t recv_buffer_size)
: m_ctx(ctx),
  m_udp_socket(ctx.ios()),
  m_remote_endpoint(boost::asio::ip::address::from_string(remote_ip), remote_port),
  m_host_endpoint(boost::asio::ip::address::from_string(host_ip), host_port),
  m_any_endpoint(boost::asio::ip::address::from_string("0.0.0.0"), host_port),
  m_recv_buffer_size(recv_buffer_size),
  m_use_multicast(false)
{
  m_remote_endpoint = remote_ip.empty() ?
    boost::asio::ip::udp::endpoint{boost::asio::ip::udp::v4(), remote_port} :
  boost::asio::ip::udp::endpoint{boost::asio::ip::address::from_string(remote_ip), remote_port};
  m_host_endpoint = host_ip.empty() ?
    boost::asio::ip::udp::endpoint{boost::asio::ip::udp::v4(), host_port} :
  boost::asio::ip::udp::endpoint{boost::asio::ip::address::from_string(host_ip), host_port};
  m_recv_buffer.resize(recv_buffer_size);
}

UdpSocket::UdpSocket(
  const drivers::common::IoContext & ctx,
  const std::string & remote_ip,
  const uint16_t remote_port,
  const std::string & host_ip,
  const uint16_t host_port) 
  : UdpSocket(ctx, remote_ip, remote_port, host_ip, host_port, m_default_recv_buffer_size)
  {}

UdpSocket::UdpSocket(
  const drivers::common::IoContext & ctx,
  const std::string & ip,
  const uint16_t port,
  const size_t recv_buffer_size)
: UdpSocket{ctx, ip, port, ip, port, recv_buffer_size}
{}

UdpSocket::UdpSocket(
  const drivers::common::IoContext & ctx,
  const std::string & ip,
  const uint16_t port)
: UdpSocket{ctx, ip, port, ip, port}
{}

UdpSocket::~UdpSocket()
{
  close();
}

std::size_t UdpSocket::send(std::vector<uint8_t> & buff)
{
  try {
    return m_udp_socket.send_to(boost::asio::buffer(buff), m_remote_endpoint);
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::send"), error.what());
    return -1;
  }
}

size_t UdpSocket::receive(std::vector<uint8_t> & buff)
{
  boost::system::error_code error;
  boost::asio::ip::udp::endpoint sender_endpoint;

  std::size_t len = m_udp_socket.receive_from(
    boost::asio::buffer(buff),
    sender_endpoint,
    0,
    error);

  if (error && error != boost::asio::error::message_size) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::receive"), error.message());
    return -1;
  }
  return len;
}

void UdpSocket::asyncSend(std::vector<uint8_t> & buff)
{
  m_udp_socket.async_send_to(
    boost::asio::buffer(buff), m_remote_endpoint,
    [this](boost::system::error_code error, std::size_t bytes_transferred)
    {
      asyncSendHandler(error, bytes_transferred);
    });
}

void UdpSocket::asyncReceive(Functor func)
{
  boost::asio::ip::udp::endpoint sender_endpoint;

  m_func = std::move(func);
  m_udp_socket.async_receive_from(
    boost::asio::buffer(m_recv_buffer),
    sender_endpoint,
    [this](boost::system::error_code error, std::size_t bytes_transferred)
    {
      asyncReceiveHandler(error, bytes_transferred);
    });
}

void UdpSocket::asyncReceiveWithSender(FunctorWithSender func)
{
  m_func_with_sender = std::move(func);
  m_udp_socket.async_receive_from(
    boost::asio::buffer(m_recv_buffer),
    sender_endpoint_,
    [this](boost::system::error_code error, std::size_t bytes_transferred)
    {
      std::cout << "asyncReceiveWithSender" << std::endl << std::endl;
      asyncReceiveHandler2(error, bytes_transferred);
    });
}

void UdpSocket::asyncSendHandler(
  const boost::system::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncSendHandler"), error.message());
  }
}

void UdpSocket::asyncReceiveHandler(
  const boost::system::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncReceiveHandler"), error.message());
    return;
  }

  if (bytes_transferred > 0 && m_func) {
    m_recv_buffer.resize(bytes_transferred);
    m_func(m_recv_buffer);
    m_recv_buffer.resize(m_recv_buffer_size);
    m_udp_socket.async_receive_from(
      boost::asio::buffer(m_recv_buffer),
      m_host_endpoint,
      [this](boost::system::error_code error, std::size_t bytes_tf)
      {
        m_recv_buffer.resize(bytes_tf);
        asyncReceiveHandler(error, bytes_tf);
      });
  }
}

void UdpSocket::asyncReceiveHandler2(
  const boost::system::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncReceiveHandler"), error.message());
    return;
  }

  if (bytes_transferred > 0 && m_func_with_sender) {
    m_recv_buffer.resize(bytes_transferred);
    m_func_with_sender(m_recv_buffer, sender_endpoint_.address().to_string());
    m_recv_buffer.resize(m_recv_buffer_size);
    m_udp_socket.async_receive_from(
      boost::asio::buffer(m_recv_buffer),
      sender_endpoint_,
      [this](boost::system::error_code error, std::size_t bytes_tf)
      {
        m_recv_buffer.resize(bytes_tf);
        asyncReceiveHandler2(error, bytes_tf);
      });
  }
}

std::string UdpSocket::remote_ip() const
{
  return m_remote_endpoint.address().to_string();
}

uint16_t UdpSocket::remote_port() const
{
  return m_remote_endpoint.port();
}

std::string UdpSocket::host_ip() const
{
  return m_host_endpoint.address().to_string();
}

uint16_t UdpSocket::host_port() const
{
  return m_host_endpoint.port();
}

void UdpSocket::open()
{
  m_udp_socket.open(boost::asio::ip::udp::v4());
  m_udp_socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
}

void UdpSocket::close()
{
  boost::system::error_code error;
  m_udp_socket.close(error);
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::close"), error.message());
  }
}

bool UdpSocket::isOpen() const
{
  return m_udp_socket.is_open();
}

void UdpSocket::bind()
{
  if (m_use_multicast) {
    m_udp_socket.bind(m_any_endpoint);
    m_udp_socket.set_option(boost::asio::ip::multicast::join_group(m_remote_endpoint.address().to_v4(), m_host_endpoint.address().to_v4()));
  } else { 
    m_udp_socket.bind(m_host_endpoint);
  }  
}

void UdpSocket::setMulticast(bool value)
{
  m_use_multicast = value;
}

bool UdpSocket::setKernelBufferSize(int32_t n_bytes) {
  boost::asio::socket_base::receive_buffer_size buffer_size_option{ n_bytes };

  try {
    m_udp_socket.set_option(buffer_size_option);
  } catch (boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::setkernelBufferSize"), error.what());
    return false;
  }

  return true;
}

}  // namespace udp_driver
}  // namespace drivers
