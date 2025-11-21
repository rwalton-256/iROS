import ARKit
import Combine

class Camera: NSObject, ObservableObject, ARSessionDelegate {
    var arSession: ARSession!
    @Published var isRunning = false
    
    var onFrameCaptured: ((ARFrame) -> Void)?
    
    
    override init() {
        self.arSession = ARSession()
        super.init()
        self.arSession.delegate = self
    }
    
    func start() {
        guard !self.isRunning else { return }
        
        let configuration = ARWorldTrackingConfiguration()

        // Optimize AR configuration for streaming
        configuration.videoFormat = ARWorldTrackingConfiguration.supportedVideoFormats
            .filter { $0.imageResolution.width == 1920 } // Use 1080p instead of 4K if available
            .first ?? ARWorldTrackingConfiguration.supportedVideoFormats[0]
        
        
        configuration.frameSemantics.insert(.sceneDepth)

        self.arSession.run(configuration)
        self.isRunning = true
        print("AR Frame Capture started")
    }
    
    func stop() {
        guard self.isRunning else { return }
        
        self.arSession.pause()
        self.isRunning = false
        print("AR Frame Capture stopped")
    }
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        self.onFrameCaptured?(frame)
        print("Frame processed and queued for send")

    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        print("AR Session Error: \(error.localizedDescription)")
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        print("AR Session interrupted")
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        print("AR Session interruption ended")
    }
}

// Helper struct for processed frame data
struct ProcessedFrame {
    let timestamp: TimeInterval
    let imageData: Data
}
