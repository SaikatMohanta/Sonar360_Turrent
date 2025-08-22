# run_turret.py
import argparse
from sonar_turret import SonarTurret

def main():
    parser = argparse.ArgumentParser(description="Run the Sonar Turret System")
    parser.add_argument("--port", type=str, default="COM3", help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--viz", type=str, choices=["2D", "3D"], default="3D", help="Visualization mode")
    args = parser.parse_args()

    print(f"Starting Sonar Turret (Port: {args.port}, Baud: {args.baud}, Viz: {args.viz})")
    turret = SonarTurret(port=args.port, baud_rate=args.baud, visualization=args.viz)
    try:
        turret.start()
    except KeyboardInterrupt:
        turret.stop()
        print("Exited gracefully")
    except Exception as e:
        print(f"[ERROR] {e}")
        turret.stop()

if __name__ == "__main__":
    main()
