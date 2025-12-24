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

// --- Simulation Constants ---
const uint32_t MIN_DATARATE_BPS = 1000000;       // 1 Mbps
const uint32_t MAX_DATARATE_BPS = 10000000;      // 10 Mbps
const double CSMA_DATARATE_MBPS = 100.0;
const double SIMULATION_DURATION_SECONDS = 21.0;

// --- Q-LEARNING AGENT IMPLEMENTATION ---
class QLearningAgent {
public:
    enum State {
        LOW_LATENCY_HIGH_TP,
        LOW_LATENCY_LOW_TP,
        HIGH_LATENCY_HIGH_TP,
        HIGH_LATENCY_LOW_TP,
        STATE_COUNT
    };

    enum Action {
        INCREASE_RATE,
        MAINTAIN_RATE,
        DECREASE_RATE,
        ACTION_COUNT
    };

    QLearningAgent(double alpha, double gamma, double epsilon);
    Action ChooseAction(State currentState);
    void UpdateQValue(State oldState, Action action, double reward, State newState);
    State GetState(double latency, double throughput);

private:
    double m_alpha;   // Learning rate
    double m_gamma;   // Discount factor
    double m_epsilon; // Exploration rate
    std::map<State, std::vector<double>> m_qTable;
    Ptr<UniformRandomVariable> m_random;
};

QLearningAgent::QLearningAgent(double alpha, double gamma, double epsilon)
    : m_alpha(alpha), m_gamma(gamma), m_epsilon(epsilon) {
    m_random = CreateObject<UniformRandomVariable>();
    for (int i = 0; i < STATE_COUNT; ++i) {
        m_qTable[(State)i] = std::vector<double>(ACTION_COUNT, 0.0);
    }
}

QLearningAgent::State QLearningAgent::GetState(double latency, double throughput) {
    bool isLowLatency = latency < 5.0; // Threshold in ms
    bool isHighThroughput = throughput > 0.5; // Threshold for ratio
    if (isLowLatency && isHighThroughput) return LOW_LATENCY_HIGH_TP;
    if (isLowLatency && !isHighThroughput) return LOW_LATENCY_LOW_TP;
    if (!isLowLatency && isHighThroughput) return HIGH_LATENCY_HIGH_TP;
    return HIGH_LATENCY_LOW_TP;
}

QLearningAgent::Action QLearningAgent::ChooseAction(State currentState) {
    if (m_random->GetValue() < m_epsilon) {
        // Exploration
        return (Action)m_random->GetInteger(0, ACTION_COUNT - 1);
    } else {
        // Exploitation
        auto it = std::max_element(m_qTable[currentState].begin(), m_qTable[currentState].end());
        return (Action)std::distance(m_qTable[currentState].begin(), it);
    }
}

void QLearningAgent::UpdateQValue(State oldState, Action action, double reward, State newState) {
    double old_value = m_qTable[oldState][action];
    double next_max = *std::max_element(m_qTable[newState].begin(), m_qTable[newState].end());

    double new_value = (1 - m_alpha) * old_value + m_alpha * (reward + m_gamma * next_max);
    m_qTable[oldState][action] = new_value;
}
// --- END OF Q-LEARNING AGENT ---

class SimulationController {
public:
    SimulationController(Ptr<OnOffApplication> clientApp, Ipv4InterfaceContainer csmaInterfaces, Ipv4InterfaceContainer serverInterfaces, std::ofstream* dataFile);
    void ControlLoop();

private:
    QLearningAgent m_agent;
    QLearningAgent::State m_currentState;
    FlowMonitorHelper m_flowmon;
    Ptr<FlowMonitor> m_monitor;
    Ptr<OnOffApplication> m_clientApp;
    Ipv4InterfaceContainer m_csmaInterfaces;
    Ipv4InterfaceContainer m_serverInterfaces;
    std::ofstream* m_dataFile;

    double CalculateReward(QLearningAgent::State state);
};

SimulationController::SimulationController(Ptr<OnOffApplication> clientApp, Ipv4InterfaceContainer csmaInterfaces, Ipv4InterfaceContainer serverInterfaces, std::ofstream* dataFile)
    : m_agent(0.1, 0.9, 0.1), // alpha, gamma, epsilon
      m_clientApp(clientApp),
      m_csmaInterfaces(csmaInterfaces),
      m_serverInterfaces(serverInterfaces),
      m_dataFile(dataFile) {
    m_monitor = m_flowmon.InstallAll();
    m_currentState = QLearningAgent::LOW_LATENCY_LOW_TP; // Initial state
}

double SimulationController::CalculateReward(QLearningAgent::State state) {
    if (state == QLearningAgent::LOW_LATENCY_HIGH_TP) return 10.0;
    if (state == QLearningAgent::LOW_LATENCY_LOW_TP) return -1.0;
    if (state == QLearningAgent::HIGH_LATENCY_HIGH_TP) return -5.0;
    if (state == QLearningAgent::HIGH_LATENCY_LOW_TP) return -10.0;
    return 0.0;
}

void SimulationController::ControlLoop() {
    m_monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(m_flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = m_monitor->GetFlowStats();

    double currentLatencyMs = 0;
    double currentThroughputRatio = 0;

    for (auto it = stats.begin(); it != stats.end(); ++it) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(it->first);
        if (t.sourceAddress == m_csmaInterfaces.GetAddress(1) && t.destinationAddress == m_serverInterfaces.GetAddress(0)) {
            if (it->second.timeLastRxPacket.GetSeconds() > it->second.timeFirstTxPacket.GetSeconds()){
                currentLatencyMs = (it->second.delaySum.GetSeconds() / it->second.rxPackets) * 1000;
                double currentBandwidthMbps = (it->second.rxBytes * 8.0) / (it->second.timeLastRxPacket.GetSeconds() - it->second.timeFirstTxPacket.GetSeconds()) / 1e6;
                currentThroughputRatio = currentBandwidthMbps / CSMA_DATARATE_MBPS;
            }
        }
    }

    QLearningAgent::State newState = m_agent.GetState(currentLatencyMs, currentThroughputRatio);
    double reward = CalculateReward(newState);

    // The learning step happens here, based on the *previous* action and the *new* state
    QLearningAgent::Action action = m_agent.ChooseAction(m_currentState);
    m_agent.UpdateQValue(m_currentState, action, reward, newState);
    m_currentState = newState;

    // Execute the chosen action
    DataRateValue currentRateValue;
    m_clientApp->GetAttribute("DataRate", currentRateValue);
    DataRate currentRate = currentRateValue.Get();

    DataRate newRate = currentRate;
    switch (action) {
        case QLearningAgent::INCREASE_RATE:
            newRate = DataRate(std::min((uint32_t)currentRate.GetBitRate() + 100000, MAX_DATARATE_BPS));
            break;
        case QLearningAgent::DECREASE_RATE:
            newRate = DataRate(std::max((uint32_t)currentRate.GetBitRate() - 100000, MIN_DATARATE_BPS));
            break;
        case QLearningAgent::MAINTAIN_RATE:
            // Do nothing
            break;
        default:
            break;
    }
    m_clientApp->SetAttribute("DataRate", DataRateValue(newRate));

    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s - RL Agent: "
        << " Latency=" << currentLatencyMs << "ms"
        << ", Throughput=" << currentThroughputRatio
        << ", State=" << newState
        << ", Reward=" << reward
        << ", Action=" << action
        << ", New Rate=" << newRate);

    *m_dataFile << Simulator::Now().GetSeconds() << "\t" << currentLatencyMs << "\t" << currentThroughputRatio << "\t" << newRate.GetBitRate() << std::endl;

    Simulator::Schedule(Seconds(1.0), &SimulationController::ControlLoop, this);
}


int main(int argc, char* argv[]) {
    // --- File Stream for Gnuplot ---
    std::ofstream dataFile;
    dataFile.open("rl_results.dat");
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
    Simulator::Schedule(Seconds(2.0), &SimulationController::ControlLoop, &controller);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    AnimationInterface anim("rl-network.xml");
    Simulator::Stop(Seconds(SIMULATION_DURATION_SECONDS));
    Simulator::Run();

    dataFile.close();

    Simulator::Destroy();
    NS_LOG_INFO("Simulation finished.");
    return 0;
}
