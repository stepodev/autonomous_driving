#ifndef PROJECT_PCAP_HPP
#define PROJECT_PCAP_HPP

/*
	Packet sniffer using libpcap library
 http://www.binarytides.com/packet-sniffer-code-c-libpcap-linux-sockets/
*/
#include<pcap.h>
#include<cstdio>
#include<cstdlib> // for exit()
#include<cstring> //for memset

#include<sys/socket.h>
#include<arpa/inet.h> // for inet_ntoa()
#include<net/ethernet.h>
#include<netinet/ip_icmp.h>	//Provides declarations for icmp header
#include<netinet/udp.h>	//Provides declarations for udp header
#include<netinet/tcp.h>	//Provides declarations for tcp header
#include<netinet/ip.h>	//Provides declarations for ip header
#include <exception>
#include <string>
#include <memory>
#include <list>
#include <functional>

namespace packetcapture {

  class pcapexception : public std::exception {
  public:
    explicit pcapexception(const std::string &msg) {
      message_ = msg.c_str();
    }

    const char *what() const throw() override {
      return message_;
    }

  private:
    const char *message_;
  };

  class Packet_info {
  public:
    ethhdr eth;
    iphdr ipheader;
    tcphdr tcpheader;
    udphdr udpheader;
    std::string payload;
  };

  class PacketCapture {
  public:
    PacketCapture();
    PacketCapture(char dev);
    PacketCapture(char dev, char filter_expr[]);
    PacketCapture(char filter_expr[]);
    void add_listener( std::function<void(std::shared_ptr<Packet_info>)> callback );

  private:
    char* dev_;       //devicename
    pcap_t *handle_; //Handle of the device that shall be sniffed
    struct bpf_program fp_;		/* The compiled filter expression */
    std::unique_ptr<char[]> filter_exp_ptr_; 	/* The filter expression */
    bpf_u_int32 mask_;		/* The netmask of our sniffing device */
    bpf_u_int32 net_;		/* The IP of our sniffing device */
    std::list<std::function< void(std::shared_ptr<Packet_info>)>> callbackmap_;

    void Init();
    void raise_event(std::shared_ptr<Packet_info> pinfo);
    static void process_packet(u_char *args, const struct pcap_pkthdr *header, const u_char *buffer);

  };
}








#endif //PROJECT_PCAP_HPP
