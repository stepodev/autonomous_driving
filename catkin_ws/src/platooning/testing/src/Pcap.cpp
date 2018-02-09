//
// Created by stepo on 1/24/18.
//


#include "Pcap.hpp"


namespace packetcapture {

  PacketCapture::PacketCapture() {
    dev_ = nullptr;
    filter_exp_ptr_ = nullptr;
    Init();
  }

  PacketCapture::PacketCapture(char *filter_expr) {
    dev_ = nullptr;
    filter_exp_ptr_ = std::unique_ptr<char[]>(filter_expr);
    Init();
  }

  PacketCapture::PacketCapture(char dev) {
    *dev_ = dev;
    filter_exp_ptr_ = nullptr;
    Init();
  }

  PacketCapture::PacketCapture(char dev, char filter_exp[]) {
    *dev_ = dev;
    filter_exp_ptr_ = std::unique_ptr<char[]>(filter_exp);
    Init();
  }

  void PacketCapture::Init() {

    char errbuf[PCAP_ERRBUF_SIZE];

    dev_ = pcap_lookupdev(errbuf);
    if (dev_ == nullptr) {
      throw pcapexception(std::string("Couldn't find default device ") + dev_ + "\n" + errbuf);;
    }

    if (pcap_lookupnet(dev_, &net_, &mask_, errbuf) == -1) {
      throw pcapexception(std::string("Can't get netmask for device ") + dev_ + "\n" + errbuf);;
    }

    handle_ = pcap_open_live(dev_, 65536, 1, 0, errbuf);
    if (handle_ == nullptr) {
      throw pcapexception(std::string("Couldn't open device ") + dev_ + "\n" + errbuf);;
    }

    if (filter_exp_ptr_ != nullptr) {
      if (pcap_compile(handle_, &fp_, filter_exp_ptr_.get(), 0, net_) == -1) {
        throw pcapexception(std::string("Couldn't parse filter ")
                            + filter_exp_ptr_.get()
                            + " \n" + pcap_geterr(handle_));;
      }
      if (pcap_setfilter(handle_, &fp_) == -1) {
        throw pcapexception(std::string("Couldn't install filter ")
                            + filter_exp_ptr_.get() + "\n"
                            + pcap_geterr(handle_));
      }
    }
    //Put the device in sniff loop
    pcap_loop(handle_, -1, &PacketCapture::process_packet, reinterpret_cast<u_char *>(this));

  }

  void PacketCapture::process_packet(u_char *user, const struct pcap_pkthdr *header, const u_char *buffer) {

    PacketCapture *caller = reinterpret_cast<PacketCapture *>(user);

    int packetsize = header->len;

    std::shared_ptr<Packet_info> pinfo = std::shared_ptr<Packet_info>(new Packet_info);

    pinfo->eth = *(struct ethhdr *) buffer;
    pinfo->ipheader = *(struct iphdr *) (buffer + sizeof(struct ethhdr));

    if (pinfo->ipheader.protocol == 6) { //udp
      pinfo->tcpheader = *(struct tcphdr *) (buffer + pinfo->ipheader.ihl + sizeof(struct ethhdr));
      int header_size = sizeof(struct ethhdr) + pinfo->ipheader.ihl + sizeof(pinfo->udpheader);
      pinfo->payload = std::string(reinterpret_cast<const char *>(buffer))
          .substr((unsigned long) buffer + header_size, (unsigned long) packetsize - header_size);
    }

    if (pinfo->ipheader.protocol == 17) { //tcp
      pinfo->tcpheader = *(struct tcphdr *) (buffer + pinfo->ipheader.ihl + sizeof(struct ethhdr));
      int header_size = sizeof(struct ethhdr) + pinfo->ipheader.ihl + pinfo->tcpheader.doff * 4;
      pinfo->payload = std::string(reinterpret_cast<const char *>(buffer))
          .substr((unsigned long) buffer + header_size, (unsigned long) packetsize - header_size);
    }

    caller->raise_event(pinfo);

//  printf("TCP : %d   UDP : %d   ICMP : %d   IGMP : %d   Others : %d   Total : %d\r", tcp , udp , icmp , igmp , others , total);
  }

  void PacketCapture::add_listener(std::function<void(std::shared_ptr<Packet_info>)> callback) {
    callbackmap_.emplace_back(callback);
  }

  void PacketCapture::raise_event(const std::shared_ptr<Packet_info> pinfo) {

    for (const auto &it : callbackmap_) {
      it(pinfo);
    }

  }
}