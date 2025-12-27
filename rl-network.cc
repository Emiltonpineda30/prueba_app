#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/csma-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FuzzyLogicNetwork");

// --- FUZZY LOGIC CONTROLLER IMPLEMENTATION ---
namespace ns3 {

struct FuzzySet {
    double a, b, c, d;
    double getMembership(double value) const;
};

class LinguisticVariable {
public:
    void addSet(std::string name, FuzzySet set);
    std::map<std::string, double> fuzzify(double crispValue);
private:
    std::map<std::string, FuzzySet> m_sets;
};

class FuzzyLogicController {
public:
    FuzzyLogicController();
    double evaluate(double bandwidth, double latency, double throughput);
private:
    LinguisticVariable m_bandwidth;
    LinguisticVariable m_latency;
    LinguisticVariable m_throughput;
    LinguisticVariable m_priority;
    void initialize();
};

double FuzzySet::getMembership(double value) const {
    if (value >= b && value <= c) return 1.0;
    if (value > a && value < b) return (value - a) / (b - a);
    if (value > c && value < d) return (d - value) / (d - c);
    return 0.0;
}

void LinguisticVariable::addSet(std::string name, FuzzySet set) {
    m_sets[name] = set;
}

std::map<std::string, double> LinguisticVariable::fuzzify(double crispValue) {
    std::map<std::string, double> fuzzyValues;
    for (auto const& [name, set] : m_sets) {
        fuzzyValues[name] = set.getMembership(crispValue);
    }
    return fuzzyValues;
}

FuzzyLogicController::FuzzyLogicController() {
    initialize();
}

void FuzzyLogicController::initialize() {
    m_bandwidth.addSet("Low", {0, 0, 20, 40});
    m_bandwidth.addSet("Medium", {20, 40, 60, 80});
    m_bandwidth.addSet("High", {60, 80, 100, 100});
    m_latency.addSet("Low", {0, 0, 10, 30});
    m_latency.addSet("Medium", {10, 30, 50, 70});
    m_latency.addSet("High", {50, 70, 100, 100});
    m_throughput.addSet("Low", {0, 0, 0.2, 0.4});
    m_throughput.addSet("Medium", {0.2, 0.4, 0.6, 0.8});
    m_throughput.addSet("High", {0.6, 0.8, 1.0, 1.0});
    m_priority.addSet("Low", {0, 0, 2, 4});
    m_priority.addSet("Medium", {2, 4, 6, 8});
    m_priority.addSet("High", {6, 8, 10, 10});
}

double FuzzyLogicController::evaluate(double bandwidth, double latency, double throughput) {
    auto bw_fuzzy = m_bandwidth.fuzzify(bandwidth);
    auto lat_fuzzy = m_latency.fuzzify(latency);
    auto thr_fuzzy = m_throughput.fuzzify(throughput);

    double high_priority_strength = std::min(bw_fuzzy["High"], lat_fuzzy["Low"]);
    double medium_priority_strength = std::max(bw_fuzzy["Medium"], thr_fuzzy["Medium"]);
    double low_priority_strength = std::max(lat_fuzzy["High"], thr_fuzzy["Low"]);

    if (bw_fuzzy["Low"] > 0 && lat_fuzzy["High"] > 0) {
        low_priority_strength = std::max(low_priority_strength, std::min(bw_fuzzy["Low"], lat_fuzzy["High"]));
    }
    if (thr_fuzzy["High"] > 0) {
        high_priority_strength = std::max(high_priority_strength, thr_fuzzy["High"]);
    }

    double numerator = 0.0;
    double denominator = 0.0;
    for (double x = 0; x <= 10; x += 0.5) {
        double clipped_low = std::min(low_priority_strength, m_priority.fuzzify(x)["Low"]);
        double clipped_medium = std::min(medium_priority_strength, m_priority.fuzzify(x)["Medium"]);
        double clipped_high = std::min(high_priority_strength, m_priority.fuzzify(x)["High"]);
        double membership_val = std::max({clipped_low, clipped_medium, clipped_high});
        numerator += x * membership_val;
        denominator += membership_val;
    }

    if (denominator == 0) return 0;
    return numerator / denominator;
}

} // namespace ns3
// --- END OF FUZZY LOGIC CONTROLLER ---

// --- Simulation Constants ---
const uint32_t MIN_DATARATE_BPS = 1000000;       // 1 Mbps
const uint32_t MAX_DATARATE_BPS = 10000000;      // 10 Mbps
const double CSMA_DATARATE_MBPS = 100.0;
const double SIMULATION_DURATION_SECONDS = 21.0;

class SimulationController {
public:
    SimulationController(Ptr<OnOffApplication> clientApp, Ipv4InterfaceContainer csmaInterfaces, Ipv4InterfaceContainer serverInterfaces, std::ofstream* dataFile);
    void ControlLoop();

private:
    FuzzyLogicController m_fuzzyController;
    FlowMonitorHelper m_flowmon;
    Ptr<FlowMonitor> m_monitor;
    Ptr<OnOffApplication> m_clientApp;
    Ipv4InterfaceContainer m_csmaInterfaces;
    Ipv4InterfaceContainer m_serverInterfaces;
    std::ofstream* m_dataFile;
};

SimulationController::SimulationController(Ptr<OnOffApplication> clientApp, Ipv4InterfaceContainer csmaInterfaces, Ipv4InterfaceContainer serverInterfaces, std::ofstream* dataFile)
    : m_clientApp(clientApp), m_csmaInterfaces(csmaInterfaces), m_serverInterfaces(serverInterfaces), m_dataFile(dataFile) {
    m_monitor = m_flowmon.InstallAll();
}

void SimulationController::ControlLoop() {
    m_monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(m_flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = m_monitor->GetFlowStats();
    for (auto it = stats.begin(); it != stats.end(); ++it) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(it->first);
        if (t.sourceAddress == m_csmaInterfaces.GetAddress(1) && t.destinationAddress == m_serverInterfaces.GetAddress(0)) {
            if (it->second.timeLastRxPacket.GetSeconds() > it->second.timeFirstTxPacket.GetSeconds()){
                double currentBandwidthMbps = (it->second.rxBytes * 8.0) / (it->second.timeLastRxPacket.GetSeconds() - it->second.timeFirstTxPacket.GetSeconds()) / 1e6;
                double currentThroughputRatio = currentBandwidthMbps / CSMA_DATARATE_MBPS;
                double currentLatencyMs = (it->second.delaySum.GetSeconds() / it->second.rxPackets) * 1000;

                double newPriority = m_fuzzyController.evaluate(currentBandwidthMbps, currentLatencyMs, currentThroughputRatio);

                uint32_t newRateBps = MIN_DATARATE_BPS + (newPriority / 10.0) * (MAX_DATARATE_BPS - MIN_DATARATE_BPS);
                DataRate newRate(newRateBps);
                m_clientApp->SetAttribute("DataRate", DataRateValue(newRate));

                NS_LOG_INFO(Simulator::Now().GetSeconds() << "s - Fuzzy Controller: Bandwidth=" << currentBandwidthMbps << "Mbps, Latency=" << currentLatencyMs << "ms, ThroughputRatio=" << currentThroughputRatio << " -> New Priority=" << newPriority << ", New Rate=" << newRate);
                *m_dataFile << Simulator::Now().GetSeconds() << "\t" << currentLatencyMs << "\t" << currentThroughputRatio << "\t" << newRateBps << std::endl;
            }
        }
    }
    Simulator::Schedule(Seconds(1.0), &SimulationController::ControlLoop, this);
}


int main(int argc, char* argv[]) {
    // --- File Stream for Gnuplot ---
    std::ofstream dataFile;
    dataFile.open("results.dat");
    dataFile << "# Time (s)\tLatency (ms)\tThroughput (Ratio)\tNew Rate (bps)" << std::endl;

    LogComponentEnable("FuzzyLogicNetwork", LOG_LEVEL_INFO);
    NodeContainer serverNode, routerNode;
    serverNode.Create(1);
    routerNode.Create(1);
    NodeContainer allNodes;
    allNodes.Add(serverNode);
    allNodes.Add(routerNode);
    InternetStackHelper stack;
    stack.Install(allNodes);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    NetDeviceContainer serverToRouterDevices = p2p.Install(serverNode.Get(0), routerNode.Get(0));
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer serverToRouterInterfaces = address.Assign(serverToRouterDevices);

    NodeContainer csmaNodes;
    csmaNodes.Add(routerNode.Get(0));
    csmaNodes.Create(10);
    stack.Install(csmaNodes);
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));
    NetDeviceContainer csmaDevices = csma.Install(csmaNodes);
    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer csmaInterfaces = address.Assign(csmaDevices);

    NodeContainer wifiApNode = routerNode;
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(5);
    stack.Install(wifiStaNodes);
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    WifiMacHelper mac;
    Ssid ssid = Ssid("ns3-wifi");
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDevices = wifi.Install(phy, mac, wifiStaNodes);
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevices = wifi.Install(phy, mac, wifiApNode);
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY", DoubleValue(0.0), "DeltaX", DoubleValue(5.0), "DeltaY", DoubleValue(10.0), "GridWidth", UintegerValue(3), "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue(Rectangle(-50, 50, -50, 50)));
    mobility.Install(wifiStaNodes);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNode);
    address.SetBase("10.1.3.0", "255.255.255.0");
    address.Assign(staDevices);
    address.Assign(apDevices);

    uint16_t port = 9;
    Address sinkAddress(InetSocketAddress(serverToRouterInterfaces.GetAddress(0), port));
    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkAddress);
    ApplicationContainer serverApps = packetSinkHelper.Install(serverNode.Get(0));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(20.0));
    OnOffHelper onoff("ns3::UdpSocketFactory", sinkAddress);
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate("5Mbps")));
    onoff.SetAttribute("PacketSize", UintegerValue(1024));
    ApplicationContainer clientApps = onoff.Install(csmaNodes.Get(1));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(20.0));
    Ptr<OnOffApplication> clientApp = DynamicCast<OnOffApplication>(clientApps.Get(0));

    // Create the simulation controller
    SimulationController controller(clientApp, csmaInterfaces, serverToRouterInterfaces, &dataFile);

    // Schedule the first call to the control loop
    Simulator::Schedule(Seconds(3.0), &SimulationController::ControlLoop, &controller);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    AnimationInterface anim("fuzzy-network.xml");
    Simulator::Stop(Seconds(SIMULATION_DURATION_SECONDS));
    Simulator::Run();

    dataFile.close(); // Close the data file

    Simulator::Destroy();
    NS_LOG_INFO("Simulation finished.");
    return 0;
}
