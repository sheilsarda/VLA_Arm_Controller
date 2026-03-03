import threading
import time
from typing import List, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
from collections import deque

# Use sim or stubbed comms based on environment
# from robot_comms_for_ur_sim import *
from robot_communication import *

class AnomalyType(Enum):
    """Types of anomalies the agent can detect."""
    EXCESSIVE_FORCE = "excessive_force"
    JOINT_OVERLOAD = "joint_overload"  # High current draw
    POSITION_DEVIATION = "position_deviation"
    VELOCITY_EXCEEDED = "velocity_exceeded"
    UNEXPECTED_RESISTANCE = "unexpected_resistance"  # High force + low velocity
    TEMPERATURE_WARNING = "temperature_warning"
    TRACKING_ERROR = "tracking_error"  # Actual vs commanded position drift; TODO: figure out how to track this in real time


@dataclass
class AnomalyEvent:
    """Record of a detected anomaly."""
    anomaly_type: AnomalyType
    timestamp: float
    joint_index: Optional[int]  # None if not joint-specific (e.g., TCP force)
    expected_value: float
    actual_value: float
    threshold: float
    message: str


@dataclass
class MonitoringThresholds:
    """
    Configurable thresholds for anomaly detection.
    TODO: these should be based on historical measurements of the robot performing the same task
    Ideally we setup BQ pub sub logging for these metrics, and then have a table where these stats 
    are all individual columns that we can slice and dice as needed.
    """

    max_tcp_force_n: float = 50.0  # Max allowable TCP force magnitude
    
    # Current thresholds (Amps) - for joint overload detection
    max_joint_current_a: List[float] = field(
        default_factory=lambda: [2.5, 2.5, 2.5, 1.5, 1.5, 1.5] 
    )
    
    # Position deviation threshold (radians)
    max_position_error_rad: float = 0.05  # ~3 degrees
    
    # Velocity threshold (rad/s)
    max_joint_velocity_rad_s: float = 3.14  # UR5e max is ~3.14 rad/s
    
    # Unexpected resistance: high force + low velocity indicates obstruction
    resistance_force_threshold_n: float = 20.0
    resistance_velocity_threshold_m_s: float = 0.01
    
    # Temperature warning (Celsius)
    max_joint_temp_c: float = 75.0  # UR5e warns at ~80C


@dataclass 
class PerformanceMetrics:
    """
    TODO: create a BQ table with this as the schema for columns we need to track
    """
    timestamp: float
    joint_positions: List[float]
    joint_velocities: List[float]
    joint_currents: List[float]
    joint_temperatures: List[float]
    tcp_pose: List[float]
    tcp_force: List[float]
    tcp_velocity: List[float]


class RobotCollisionDetectionAgent:
    """
    Class that monitors and logs the robot's performance metrics, such as joint positions, 
    velocities, and applied forces, during the pick-and-place operation. The
    system should also detect any deviations from expected values and initiate corrective actions or
    safety stops if required. 
    
    Ensure that the system accounts for potential hardware issues, such as
    joint overloading or unexpected resistance, and integrates with the fault recovery mechanism.
    
    Usage:
        agent = RobotCollisionDetectionAgent()
        agent.set_anomaly_callback(my_handler)  # Optional: custom handling
        agent.start_monitoring()
        
        # Check for issues:
        if agent.has_active_anomalies():
            agent.request_safety_stop()
        
        # Cleanup:
        agent.stop_monitoring()
    """
    
    # Sampling rate for monitoring loop (Hz)
    MONITORING_RATE_HZ = 50  # 50Hz = 20ms between samples
    
    # History buffer size for metrics logging
    METRICS_HISTORY_SIZE = 500  # ~10 seconds at 50Hz
    
    def __init__(
        self, 
        thresholds: Optional[MonitoringThresholds] = None,
        auto_safety_stop: bool = True
    ):

        self.thresholds = thresholds or MonitoringThresholds()
        self.auto_safety_stop = auto_safety_stop
        
        # Monitoring state
        self._monitoring = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        self._active_anomalies: List[AnomalyEvent] = []
        self._anomaly_history: deque = deque(maxlen=100)
        
        # Performance metrics history for logging/debugging
        self._metrics_history: deque = deque(maxlen=self.METRICS_HISTORY_SIZE)
        
        # Callback for external anomaly handling (e.g., notify controller)
        self._anomaly_callback: Optional[Callable[[AnomalyEvent], None]] = None
        
        # Safety stop requested flag
        self._safety_stop_requested = False
    
    # ==================== Public API ====================
    
    def start_monitoring(self) -> None:
        """Start the background monitoring thread."""
        if self._monitoring:
            return
        
        self._monitoring = True
        self._safety_stop_requested = False
        self._monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self._monitor_thread.start()
        print("[CollisionAgent] Monitoring started")
    
    def stop_monitoring(self) -> None:
        """Stop the background monitoring thread."""
        self._monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1.0)
            self._monitor_thread = None
        print("[CollisionAgent] Monitoring stopped")
    
    
    def set_anomaly_callback(self, callback: Callable[[AnomalyEvent], None]) -> None:
        """
        Register a callback to be invoked when anomalies are detected.
        
        Args:
            callback: Function that takes an AnomalyEvent
        """
        self._anomaly_callback = callback
    
    def has_active_anomalies(self) -> bool:
        """Check if there are any unresolved anomalies."""
        with self._lock:
            return len(self._active_anomalies) > 0
    
    def get_active_anomalies(self) -> List[AnomalyEvent]:
        """Get list of current active anomalies."""
        with self._lock:
            return list(self._active_anomalies)
    
    def clear_anomalies(self) -> None:
        """Clear active anomalies (e.g., after recovery)."""
        with self._lock:
            self._active_anomalies.clear()
            self._safety_stop_requested = False
    
    def request_safety_stop(self) -> None:
        """Request an immediate safety stop."""
        self._safety_stop_requested = True
        self._execute_safety_stop()
    
    def is_safety_stop_requested(self) -> bool:
        """Check if a safety stop has been requested."""
        return self._safety_stop_requested
    
    def get_metrics_history(self) -> List[PerformanceMetrics]:
        """Get recent performance metrics for logging/debugging."""
        with self._lock:
            return list(self._metrics_history)
    
    def get_latest_metrics(self) -> Optional[PerformanceMetrics]:
        """Get the most recent performance metrics snapshot."""
        with self._lock:
            if self._metrics_history:
                return self._metrics_history[-1]
            return None
    
    # ==================== Internal Methods ====================
    
    def _monitoring_loop(self) -> None:
        """Background thread that continuously monitors robot state."""
        sample_interval = 1.0 / self.MONITORING_RATE_HZ
        
        while self._monitoring:
            loop_start = time.time()
            
            try:
                # Sample current robot state
                metrics = self._sample_metrics()
                
                # Store in history
                with self._lock:
                    self._metrics_history.append(metrics)
                
                # Run anomaly detection checks
                self._check_for_anomalies(metrics)
                
            except Exception as e:
                print(f"[CollisionAgent] Error in monitoring loop: {e}")
            
            # Maintain consistent sample rate
            elapsed = time.time() - loop_start
            sleep_time = sample_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _sample_metrics(self) -> PerformanceMetrics:
        """Sample all relevant robot metrics."""
        
        return PerformanceMetrics(
            timestamp=time.time(),
            joint_positions=get_angular_pose(),
            joint_velocities=get_angular_velocity(),
            joint_currents=self._get_joint_currents_safe(),
            joint_temperatures=self._get_joint_temperatures_safe(),
            tcp_pose=get_base_cartesian_pose(),
            tcp_force=get_cartesian_base_force(),
            tcp_velocity=get_base_cartesian_velocity(),
        )
    
    def _get_joint_currents_safe(self) -> List[float]:
        """Get joint currents, with fallback if not available."""
        # The sim interface (URRobotState) tracks _joint_currents
        # Stubbed interface doesn't have this - return zeros
        try:
            # This would come from the real-time interface in robot_comms_for_ur_sim
            # For now, return placeholder
            return [0.0] * 6
        except:
            return [0.0] * 6
    
    def _get_joint_temperatures_safe(self) -> List[float]:
        """Get joint temperatures, with fallback if not available."""
        try:
            return [0.0] * 6
        except:
            return [0.0] * 6
    
    def _check_for_anomalies(self, metrics: PerformanceMetrics) -> None:
        """Run all anomaly detection checks on the current metrics."""
        
        # 1. Check TCP force magnitude (collision detection)
        self._check_excessive_force(metrics)
        
        # 2. Check joint currents (overload detection)
        self._check_joint_overload(metrics)
        
        # 3. Check position tracking error; TODO for now
        
        # 4. Check for unexpected resistance (high force + low velocity)
        self._check_unexpected_resistance(metrics)
        
        # 5. Check joint velocities
        self._check_velocity_limits(metrics)
        
        # 6. Check temperatures
        self._check_temperature_limits(metrics)
    
    def _check_excessive_force(self, metrics: PerformanceMetrics) -> None:
        """Check if TCP force exceeds safe threshold."""
        fx, fy, fz = metrics.tcp_force[:3]
        force_magnitude = (fx**2 + fy**2 + fz**2) ** 0.5
        
        if force_magnitude > self.thresholds.max_tcp_force_n:
            self._report_anomaly(AnomalyEvent(
                anomaly_type=AnomalyType.EXCESSIVE_FORCE,
                timestamp=metrics.timestamp,
                joint_index=None,
                expected_value=0.0,
                actual_value=force_magnitude,
                threshold=self.thresholds.max_tcp_force_n,
                message=f"TCP force {force_magnitude:.1f}N exceeds limit {self.thresholds.max_tcp_force_n}N"
            ))
    
    def _check_joint_overload(self, metrics: PerformanceMetrics) -> None:
        """Check if any joint current exceeds its limit."""
        for i, current in enumerate(metrics.joint_currents):
            limit = self.thresholds.max_joint_current_a[i]
            if abs(current) > limit:
                self._report_anomaly(AnomalyEvent(
                    anomaly_type=AnomalyType.JOINT_OVERLOAD,
                    timestamp=metrics.timestamp,
                    joint_index=i,
                    expected_value=0.0,
                    actual_value=current,
                    threshold=limit,
                    message=f"Joint {i+1} current {current:.2f}A exceeds limit {limit}A"
                ))
    
    def _check_unexpected_resistance(self, metrics: PerformanceMetrics) -> None:
        """Detect obstruction: high force but low/zero velocity."""
        fx, fy, fz = metrics.tcp_force[:3]
        force_magnitude = (fx**2 + fy**2 + fz**2) ** 0.5
        
        vx, vy, vz = metrics.tcp_velocity[:3]
        velocity_magnitude = (vx**2 + vy**2 + vz**2) ** 0.5
        
        if (force_magnitude > self.thresholds.resistance_force_threshold_n and 
            velocity_magnitude < self.thresholds.resistance_velocity_threshold_m_s):
            self._report_anomaly(AnomalyEvent(
                anomaly_type=AnomalyType.UNEXPECTED_RESISTANCE,
                timestamp=metrics.timestamp,
                joint_index=None,
                expected_value=0.0,
                actual_value=force_magnitude,
                threshold=self.thresholds.resistance_force_threshold_n,
                message=f"Unexpected resistance: force={force_magnitude:.1f}N, velocity={velocity_magnitude:.4f}m/s"
            ))
    
    def _check_velocity_limits(self, metrics: PerformanceMetrics) -> None:
        """Check if any joint velocity exceeds safe limits."""
        for i, velocity in enumerate(metrics.joint_velocities):
            if abs(velocity) > self.thresholds.max_joint_velocity_rad_s:
                self._report_anomaly(AnomalyEvent(
                    anomaly_type=AnomalyType.VELOCITY_EXCEEDED,
                    timestamp=metrics.timestamp,
                    joint_index=i,
                    expected_value=0.0,
                    actual_value=velocity,
                    threshold=self.thresholds.max_joint_velocity_rad_s,
                    message=f"Joint {i+1} velocity {velocity:.2f}rad/s exceeds limit"
                ))
    
    def _check_temperature_limits(self, metrics: PerformanceMetrics) -> None:
        """Check if any joint temperature is too high."""
        for i, temp in enumerate(metrics.joint_temperatures):
            if temp > self.thresholds.max_joint_temp_c:
                self._report_anomaly(AnomalyEvent(
                    anomaly_type=AnomalyType.TEMPERATURE_WARNING,
                    timestamp=metrics.timestamp,
                    joint_index=i,
                    expected_value=self.thresholds.max_joint_temp_c,
                    actual_value=temp,
                    threshold=self.thresholds.max_joint_temp_c,
                    message=f"Joint {i+1} temperature {temp:.1f}C exceeds limit"
                ))
    
    def _report_anomaly(self, event: AnomalyEvent) -> None:
        """Handle a detected anomaly."""
        with self._lock:
            # Avoid duplicate spam: check if same type anomaly recently reported
            recent_cutoff = time.time() - 0.5  # Debounce: 500ms
            for existing in self._active_anomalies:
                if (existing.anomaly_type == event.anomaly_type and 
                    existing.joint_index == event.joint_index and
                    existing.timestamp > recent_cutoff):
                    return  # Already reported recently
            
            self._active_anomalies.append(event)
            self._anomaly_history.append(event)
        
        print(f"[CollisionAgent] ANOMALY: {event.message}")
        
        # Invoke callback if registered
        if self._anomaly_callback:
            try:
                self._anomaly_callback(event)
            except Exception as e:
                print(f"[CollisionAgent] Callback error: {e}")
        
        # Auto safety stop for critical anomalies
        critical_types = {
            AnomalyType.EXCESSIVE_FORCE,
            AnomalyType.JOINT_OVERLOAD,
            AnomalyType.UNEXPECTED_RESISTANCE
        }
        if self.auto_safety_stop and event.anomaly_type in critical_types:
            self.request_safety_stop()
    
    def _execute_safety_stop(self) -> None:
        """Execute an immediate safety stop."""
        print("[CollisionAgent] EXECUTING SAFETY STOP")
        try:
            # Set velocities to zero with high deceleration
            set_angular_velocity([0.0] * 6)
            # TODO: hook this up to a dedicated stop command if available
        except Exception as e:
            print(f"[CollisionAgent] Error during safety stop: {e}")
