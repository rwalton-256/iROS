import SwiftUI
import ARKit
import Foundation

// MARK: - Main App View
struct ContentView: View {
    @StateObject private var tcpManager = TCPConnectionManager()
    @StateObject private var camera = Camera()
    @State private var hostAddress = "10.0.0.24"
    @State private var port = "8889"
    @State private var connectionName = "iPhone"
    @State private var messageText = ""
    
    // Create CIContext once and reuse it
    private let ciContext = CIContext(options: [
        .useSoftwareRenderer: false,
        .cacheIntermediates: false  // Don't cache intermediates
    ])
    
    private let processingQueue = DispatchQueue(label: "com.app.imageProcessing", qos: .userInitiated)

    
    var body: some View {
        NavigationView {
            VStack(spacing: 20) {
                // Connection Section
                VStack(alignment: .leading, spacing: 10) {
                    Text("Connection")
                        .font(.headline)
                    
                    HStack {
                        Text("Host:")
                            .frame(width: 50, alignment: .leading)
                        TextField("IP Address", text: $hostAddress)
                            .textFieldStyle(RoundedBorderTextFieldStyle())
                            .autocapitalization(.none)
                            .keyboardType(.decimalPad)
                    }
                    
                    HStack {
                        Text("Port:")
                            .frame(width: 50, alignment: .leading)
                        TextField("Port", text: $port)
                            .textFieldStyle(RoundedBorderTextFieldStyle())
                            .keyboardType(.numberPad)
                    }
                    HStack {
                        Text("Connection Name:")
                            .frame(width: 100, alignment: .leading)
                        TextField("Connection Name", text: $connectionName)
                            .textFieldStyle(RoundedBorderTextFieldStyle())
                            .keyboardType(.default)
                    }
                    
                    HStack {
                        Button(tcpManager.isConnected ? "Disconnect" : "Connect") {
                            if tcpManager.isConnected {
                                tcpManager.disconnect()
                                if camera.isRunning {
                                    camera.stop()
                                }
                            } else {
                                if let portNum = UInt16(port) {
                                    tcpManager.connect(to: hostAddress, port: portNum)
                                }
                            }
                        }
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(tcpManager.isConnected ? Color.red : Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(10)
                    }
                    
                    Text(tcpManager.statusMessage)
                        .font(.caption)
                        .foregroundColor(tcpManager.isConnected ? .green : .gray)
                    
                    Button(camera.isRunning ? "Stop" : "Start") {
                        guard tcpManager.isConnected else { return }
                        if camera.isRunning {
                            camera.stop()
                        } else {
                            camera.start()
                        }
                    }
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(!tcpManager.isConnected ? Color.gray : camera.isRunning ? Color.red : Color.blue)
                    .foregroundColor(.white)
                    .cornerRadius(10)
                }
                .padding()
                .background(Color(.systemGray6))
                .cornerRadius(10)
            }
            .navigationTitle("TCP Socket")
            .onChange(of: tcpManager.isConnected) { oldValue, newValue in
                if newValue == true {
                    if let nameData = connectionName.data(using: .utf8) {
                        tcpManager.sendPacket(messageType: 0, timestamp: Date().timeIntervalSince1970, payload: nameData)
                        print("Connection name sent: \(connectionName)")
                    }
                }
            }
            .onAppear {
                camera.onFrameCaptured = { [weak tcpManager] frame in
                    guard let tcpManager = tcpManager else { return }
                    
                    // FIX 1: Extract data IMMEDIATELY on AR thread before async dispatch
                    // This prevents retaining the entire ARFrame
                    let pixelBuffer = frame.capturedImage
                    let timestamp = frame.timestamp + (Date().timeIntervalSince1970 - ProcessInfo.processInfo.systemUptime)
                    
                    // FIX 2: Extract depth data before async dispatch
                    guard let depth = frame.sceneDepth else { return }
                    let depthMap = depth.depthMap
                    
                    // Lock and copy depth data synchronously
                    CVPixelBufferLockBaseAddress(depthMap, .readOnly)
                    let depthDataSize = CVPixelBufferGetDataSize(depthMap)
                    guard let depthBaseAddress = CVPixelBufferGetBaseAddress(depthMap) else {
                        CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)
                        return
                    }
                    
                    // Create a copy of depth data
                    let depthData = Data(bytes: depthBaseAddress, count: depthDataSize)
                    CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)
                    
                    // Now dispatch to background with copied data only
                    processingQueue.async { [weak tcpManager] in
                        guard let tcpManager = tcpManager else { return }
                        
                        // FIX 3: Use autoreleasepool to release memory each iteration
                        autoreleasepool {
                            let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
                            
                            // FIX 4: Create CGImage with lower memory footprint
                            guard let cgImage = ciContext.createCGImage(
                                ciImage,
                                from: ciImage.extent,
                                format: .RGBA8,  // Explicit format
                                colorSpace: CGColorSpaceCreateDeviceRGB()
                            ) else {
                                return
                            }
                            
                            // Convert to JPEG
                            guard let imageData = UIImage(cgImage: cgImage).jpegData(compressionQuality: 0.5) else {
                                return
                            }
                            
                            print("Sending Payload")
                            tcpManager.sendPacket(messageType: 1, timestamp: timestamp, payload: imageData)
                            tcpManager.sendPacket(messageType: 2, timestamp: timestamp, payload: depthData)
                        }
                    }
                }
            }
        }
    }
}

// MARK: - App Entry Point
@main
struct TCPSocketApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
    }
}
