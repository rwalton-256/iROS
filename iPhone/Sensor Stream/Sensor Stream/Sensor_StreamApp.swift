import Network
import SwiftUI
import Combine
import Foundation



// MARK: - TCP Connection Manager
class TCPConnectionManager: ObservableObject {
    @Published var isConnected = false
    @Published var messages: [Message] = []
    @Published var statusMessage = "Not connected"
    
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "TCPQueue")
    
    struct Message: Identifiable {
        let id = UUID()
        let text: String
        let isOutgoing: Bool
        let timestamp = Date()
    }
    
    func connect(to host: String, port: UInt16) {
        let endpoint = NWEndpoint.hostPort(
            host: NWEndpoint.Host(host),
            port: NWEndpoint.Port(rawValue: port)!
        )
        
        connection = NWConnection(to: endpoint, using: .tcp)
        
        connection?.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                self?.handleStateChange(state)
            }
        }
        
        connection?.start(queue: queue)
        receiveData()
    }
    
    func disconnect() {
        connection?.cancel()
        connection = nil
        DispatchQueue.main.async {
            self.isConnected = false
            self.statusMessage = "Disconnected"
        }
    }
    
    func send(_ message: String) {
        guard isConnected else { return }
        
        let data = (message + "\n").data(using: .utf8)!
        connection?.send(content: data, completion: .contentProcessed { [weak self] error in
            if let error = error {
                print("Send error: \(error)")
            } else {
                DispatchQueue.main.async {
                    self?.messages.append(Message(text: message, isOutgoing: true))
                }
            }
        })
    }
    
    func sendPacket(messageType: Int, timestamp: TimeInterval, payload: Data){
        guard isConnected else { return }
        

        
        let timestampSecondsValue = Int32(timestamp)
        let fractionalPart = timestamp - Double(timestampSecondsValue)
        let timestampNanosecondsValue = UInt32(fractionalPart * 1_000_000_000)
        
        // Now convert to big-endian for network transmission
        var messageID = UInt32(messageType).bigEndian
        var payloadLength = UInt32(payload.count).bigEndian
        var timestampSeconds = timestampSecondsValue.bigEndian
        var timestampNanoseconds = timestampNanosecondsValue.bigEndian
        
        
        let messageIDData = Data(bytes: &messageID, count: 4)
        let payloadLengthData = Data(bytes: &payloadLength, count: 4)
        let timestampSecondsData = Data(bytes: &timestampSeconds, count:4)
        let timestampNanosecondsData = Data(bytes: &timestampNanoseconds, count:4)
        let reservedData = Data(repeating: 0, count: 16)
        
        var packetData = Data()
        packetData.append(messageIDData)
        packetData.append(payloadLengthData)
        packetData.append(timestampSecondsData)
        packetData.append(timestampNanosecondsData)
        packetData.append(reservedData)
        packetData.append(payload)
        
        
        connection?.send(content: packetData, completion: .contentProcessed { [weak self] error in
            if let error = error {
                print("Send error: \(error)")
            } else {
            }
        })
        
        
    }
        
    func sendData(_ message: Data) {
        guard isConnected else { return }
        
        connection?.send(content: message, completion: .contentProcessed { [weak self] error in
            if let error = error {
                print("Send error: \(error)")
            } else {
            }
        })
    }
    
    private func handleStateChange(_ state: NWConnection.State) {
        switch state {
        case .ready:
            isConnected = true
            statusMessage = "Connected"
        case .waiting(let error):
            isConnected = false
            statusMessage = "Waiting: \(error.localizedDescription)"
        case .failed(let error):
            isConnected = false
            statusMessage = "Failed: \(error.localizedDescription)"
            connection?.cancel()
        case .cancelled:
            isConnected = false
            statusMessage = "Cancelled"
        default:
            statusMessage = "Connecting..."
        }
    }
    
    private func receiveData() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] data, _, isComplete, error in
            if let data = data, !data.isEmpty {
                if let message = String(data: data, encoding: .utf8) {
                    let lines = message.components(separatedBy: "\n").filter { !$0.isEmpty }
                    DispatchQueue.main.async {
                        for line in lines {
                            self?.messages.append(Message(text: line, isOutgoing: false))
                            print(line)
                        }
                    }
                }
            }
            
            if let error = error {
                print("Receive error: \(error)")
                return
            }
            
            if !isComplete {
                self?.receiveData()
            }
        }
    }
}

