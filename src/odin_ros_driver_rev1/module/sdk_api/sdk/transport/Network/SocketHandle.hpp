#pragma once

// Platform socket-handle normalization shared by the TCP/UDP transports
// (global namespace, matching the historical UdpTransport.hpp usage).
//
// SOCKET is an unsigned 64-bit type on Win64, so storing it in an int (the
// old TcpTransport layout) both truncates the handle and compares against
// the wrong "invalid" value (INVALID_SOCKET is ~0, not -1).
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocketHandle = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
constexpr SocketHandle kInvalidSocketHandle = -1;
#endif
