"""
MATLAB Engine Interface for Control Simulation
Handles MATLAB engine startup, control algorithm execution, and result retrieval
"""

import matlab.engine
import numpy as np
import json
import os
from typing import Dict, Any, Optional, List, Tuple
import threading
import time


class MATLABControlSimulator:
    """
    Interface to MATLAB for running control simulations.
    Supports custom control algorithms with user-defined parameters.
    """

    def __init__(self, matlab_script_path: str = None):
        """
        Initialize MATLAB engine interface
        
        Args:
            matlab_script_path: Path to MATLAB scripts directory
        """
        self.engine = None
        self.engine_lock = threading.Lock()
        self.is_running = False
        
        # Set MATLAB script path
        if matlab_script_path is None:
            self.matlab_script_path = os.path.join(
                os.path.dirname(__file__), 'matlab_scripts'
            )
        else:
            self.matlab_script_path = matlab_script_path
            
        # Results storage
        self.last_results = None
        
    def start_engine(self) -> bool:
        """
        Start MATLAB engine (can take several seconds)
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            print("Starting MATLAB engine...")
            self.engine = matlab.engine.start_matlab()
            
            # Add MATLAB scripts to path
            self.engine.addpath(self.matlab_script_path, nargout=0)
            
            print("MATLAB engine started successfully")
            return True
        except Exception as e:
            print(f"Error starting MATLAB engine: {e}")
            return False
    
    def stop_engine(self):
        """Stop MATLAB engine"""
        with self.engine_lock:
            if self.engine is not None:
                try:
                    self.engine.quit()
                    print("MATLAB engine stopped")
                except Exception as e:
                    print(f"Error stopping MATLAB engine: {e}")
                finally:
                    self.engine = None
    
    def is_engine_running(self) -> bool:
        """Check if MATLAB engine is running"""
        return self.engine is not None
    
    def run_control_simulation(
        self,
        algorithm_name: str,
        parameters: Dict[str, float],
        sim_time: float = 10.0,
        initial_state: Optional[List[float]] = None,
        reference_signal: Optional[str] = "step",
        reference_amplitude: float = 1.0
    ) -> Dict[str, Any]:
        """
        Run a control simulation with specified parameters
        
        Args:
            algorithm_name: Name of control algorithm ('pid', 'lqr', 'mpc', etc.)
            parameters: Dictionary of control parameters
            sim_time: Simulation duration in seconds
            initial_state: Initial system state
            reference_signal: Type of reference ('step', 'ramp', 'sine')
            reference_amplitude: Amplitude of reference signal
            
        Returns:
            Dictionary with simulation results:
                - time: time vector
                - states: system states over time
                - control: control inputs over time
                - reference: reference signal
                - error: tracking error
                - metrics: performance metrics (overshoot, settling time, etc.)
        """
        if not self.is_engine_running():
            raise RuntimeError("MATLAB engine not started. Call start_engine() first.")
        
        with self.engine_lock:
            self.is_running = True
            
            try:
                # Convert parameters to MATLAB format
                matlab_params = self._python_to_matlab(parameters)
                
                # Prepare initial state
                if initial_state is None:
                    matlab_initial_state = matlab.double([])
                else:
                    matlab_initial_state = matlab.double(initial_state)
                
                # Call MATLAB simulation function
                print(f"Running {algorithm_name} simulation...")
                
                result = self.engine.run_control_sim(
                    algorithm_name,
                    matlab_params,
                    sim_time,
                    matlab_initial_state,
                    reference_signal,
                    reference_amplitude,
                    nargout=1
                )
                
                # Convert MATLAB results to Python
                results = self._matlab_to_python(result)
                
                # Compute additional metrics
                results['metrics'] = self._compute_metrics(results)
                
                self.last_results = results
                
                print(f"Simulation complete. Metrics: {results['metrics']}")
                
                return results
                
            except Exception as e:
                print(f"Error running simulation: {e}")
                raise
            finally:
                self.is_running = False
    
    def _python_to_matlab(self, data: Dict[str, float]) -> matlab.double:
        """Convert Python dictionary to MATLAB struct-like format"""
        # For simplicity, pass as array of values
        # MATLAB function will interpret based on algorithm
        return matlab.double(list(data.values()))
    
    def _matlab_to_python(self, matlab_result: Dict) -> Dict[str, Any]:
        """Convert MATLAB result to Python dictionary"""
        result = {}
        
        for key in matlab_result.keys():
            value = matlab_result[key]
            
            # Convert MATLAB arrays to numpy
            if hasattr(value, '_data'):
                result[key] = np.array(value._data).reshape(value.size)
            else:
                result[key] = value
        
        return result
    
    def _compute_metrics(self, results: Dict[str, Any]) -> Dict[str, float]:
        """
        Compute control performance metrics
        
        Args:
            results: Simulation results with time, states, reference, error
            
        Returns:
            Dictionary of metrics
        """
        metrics = {}
        
        try:
            time = np.array(results['time']).flatten()
            output = np.array(results['states']).flatten()
            reference = np.array(results['reference']).flatten()
            error = np.array(results['error']).flatten()
            control = np.array(results['control']).flatten()
            
            # Rise time (10% to 90%)
            final_value = reference[-1]
            rise_10 = 0.1 * final_value
            rise_90 = 0.9 * final_value
            
            idx_10 = np.where(output >= rise_10)[0]
            idx_90 = np.where(output >= rise_90)[0]
            
            if len(idx_10) > 0 and len(idx_90) > 0:
                metrics['rise_time'] = time[idx_90[0]] - time[idx_10[0]]
            else:
                metrics['rise_time'] = float('nan')
            
            # Settling time (2% criterion)
            settling_band = 0.02 * abs(final_value)
            settled_idx = np.where(
                np.abs(output - final_value) <= settling_band
            )[0]
            
            if len(settled_idx) > 0:
                # Check if it stays settled
                for idx in settled_idx:
                    if np.all(
                        np.abs(output[idx:] - final_value) <= settling_band
                    ):
                        metrics['settling_time'] = time[idx]
                        break
                else:
                    metrics['settling_time'] = float('nan')
            else:
                metrics['settling_time'] = float('nan')
            
            # Overshoot
            max_output = np.max(output)
            if final_value != 0:
                metrics['overshoot_percent'] = (
                    (max_output - final_value) / abs(final_value) * 100
                )
            else:
                metrics['overshoot_percent'] = 0.0
            
            # Steady-state error
            metrics['steady_state_error'] = abs(output[-1] - reference[-1])
            
            # RMS error
            metrics['rms_error'] = np.sqrt(np.mean(error**2))
            
            # Peak control effort
            metrics['peak_control'] = np.max(np.abs(control))
            
            # Control effort (integral of squared control)
            dt = time[1] - time[0] if len(time) > 1 else 1.0
            metrics['control_effort'] = np.sum(control**2) * dt
            
            # IAE (Integral Absolute Error)
            metrics['iae'] = np.sum(np.abs(error)) * dt
            
            # ISE (Integral Squared Error)
            metrics['ise'] = np.sum(error**2) * dt
            
            # ITAE (Integral Time Absolute Error)
            metrics['itae'] = np.sum(time * np.abs(error)) * dt
            
        except Exception as e:
            print(f"Error computing metrics: {e}")
            # Return default metrics on error
            metrics = {
                'rise_time': float('nan'),
                'settling_time': float('nan'),
                'overshoot_percent': float('nan'),
                'steady_state_error': float('nan'),
                'rms_error': float('nan'),
                'peak_control': float('nan'),
                'control_effort': float('nan'),
                'iae': float('nan'),
                'ise': float('nan'),
                'itae': float('nan')
            }
        
        return metrics
    
    def get_available_algorithms(self) -> List[str]:
        """Get list of available control algorithms"""
        algorithms = []
        
        # Check for .m files in matlab_scripts directory
        if os.path.exists(self.matlab_script_path):
            for file in os.listdir(self.matlab_script_path):
                if file.endswith('_controller.m'):
                    algo_name = file.replace('_controller.m', '')
                    algorithms.append(algo_name)
        
        return algorithms if algorithms else ['pid', 'lqr', 'mpc']
    
    def __enter__(self):
        """Context manager entry"""
        self.start_engine()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop_engine()
        return False
