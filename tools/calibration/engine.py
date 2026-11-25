"""
Deep CalibrationEngine module coordinating pre-flight hardware safety and closed-loop kinematic convergence.
"""

from typing import List, Optional

from tools.calibration.models import (
    CalibrationProfileData,
    CalibrationReport,
    DiagnosticResult,
    SafetyGate,
    SafetyGateError,
    TrialRecord,
    U0Report,
    U0SubtestReport,
    U1Result,
    U2Result,
    U3Result,
    U4Result,
    U4SubResult,
)
from tools.calibration.formulas import (
    calculate_motor_trim_correction,
    calculate_wheel_diameter_correction,
    check_u1_tolerances,
    check_u2_tolerances,
    check_u3_tolerances,
    check_u4_tolerances,
)
from tools.calibration.adapters.observation import ObservationAdapter
from tools.calibration.adapters.reporter import NullReporter, ReporterAdapter
from tools.calibration.adapters.transport import TransportAdapter


class CalibrationEngine:
    """
    Deep domain module for autonomous robot calibration.
    Orchestrates hardware pre-flight verification, strict safety gating,
    closed-loop iterative kinematic parameter tuning, and non-volatile EEPROM updates.
    """
    def __init__(self,
                 transport: TransportAdapter,
                 observation: ObservationAdapter,
                 reporter: Optional[ReporterAdapter] = None) -> None:
        self.transport: TransportAdapter = transport
        self.observation: ObservationAdapter = observation
        self.reporter: ReporterAdapter = reporter or NullReporter()
        self.safety_gate: SafetyGate = SafetyGate()
        self.last_u0_report: Optional[U0Report] = None

    def is_motion_permitted(self) -> bool:
        """Returns True if Stage U0 pre-flight hardware diagnostics passed 100%."""
        return self.safety_gate.is_motion_permitted()

    def get_safety_gate(self) -> SafetyGate:
        """Returns the active SafetyGate state."""
        return self.safety_gate

    def get_active_profile(self) -> CalibrationProfileData:
        """Reads active calibration profile from microcontroller EEPROM."""
        return self.transport.read_profile()

    def write_profile(self, profile: CalibrationProfileData) -> bool:
        """Writes calibrated profile parameters to microcontroller EEPROM."""
        return self.transport.write_profile(profile)

    def reset_profile(self) -> bool:
        """Resets EEPROM profile to factory compile-time defaults."""
        return self.transport.reset_profile()

    def run_preflight(self) -> U0Report:
        """
        Executes Stage U0: Pre-Flight Hardware & Wiring Diagnostics.
        Hard-gates kinematic calibration: 100% of components must pass before motion is permitted.
        """
        report = U0Report()
        self.reporter.on_stage_start("Stage U0", "Pre-Flight Hardware & Wiring Diagnostics")

        # Step 1: Probe sensors via !DIAG:ALL
        try:
            diag: DiagnosticResult = self.transport.probe_diagnostics()
        except Exception as e:
            st = U0SubtestReport("Koneksi & Sensor Bus", False, f"Gagal membaca diagnostik sensor: {e}",
                                 ["Cek kabel USB, baudrate 115200, dan daya baterai robot."])
            report.add_subtest(st)
            self.safety_gate.update_report(report)
            self.last_u0_report = report
            self.reporter.on_subtest_result(st.name, False, st.details, st.pin_hints)
            self.reporter.on_hard_gate_status(False, report)
            return report

        # U0.1: I2C & MPU6050 WHO_AM_I
        if diag.mpu_ok:
            st_mpu = U0SubtestReport("U0.1 MPU6050", True, "I2C 0x68 responsif (WHO_AM_I valid)")
            report.u01_mpu_passed = True
        else:
            st_mpu = U0SubtestReport("U0.1 MPU6050", False, "WHO_AM_I gagal / I2C bus error", [
                "Periksa kabel jumper MPU6050: VCC->5V, GND->GND.",
                "Periksa pin I2C Arduino Uno: SDA terhubung ke Pin A4, SCL terhubung ke Pin A5.",
            ])
            report.u01_mpu_passed = False
        report.add_subtest(st_mpu)
        self.reporter.on_subtest_result(st_mpu.name, st_mpu.passed, st_mpu.details, st_mpu.pin_hints)

        # U0.2: Ultrasonic Sonar HC-SR04
        if 2.0 <= diag.sonar_dist_cm <= 400.0:
            st_sonar = U0SubtestReport("U0.2 Sonar", True, f"Jarak valid: {diag.sonar_dist_cm:.1f} cm (bebas timeout)")
            report.u02_sonar_passed = True
        else:
            st_sonar = U0SubtestReport("U0.2 Sonar", False, f"Timeout sonar ({diag.sonar_dist_cm:.1f} cm)", [
                "Periksa pin Sonar: Trig terhubung ke Pin A0, Echo terhubung ke Pin A1.",
                "Pastikan sensor mendapat suplai 5V bersih dan tidak terhalang kabel.",
            ])
            report.u02_sonar_passed = False
        report.add_subtest(st_sonar)
        self.reporter.on_subtest_result(st_sonar.name, st_sonar.passed, st_sonar.details, st_sonar.pin_hints)

        # U0.3: FC-51 IR Lateral Proximity Sensors
        ir_verified = self.observation.verify_ir_sensors(diag.ir_left, diag.ir_right)
        ir_details = f"IR Kiri={diag.ir_left}, IR Kanan={diag.ir_right}"
        if ir_verified:
            st_ir = U0SubtestReport("U0.3 IR Proximity", True, f"Terverifikasi ({ir_details})")
            report.u03_ir_passed = True
        else:
            st_ir = U0SubtestReport("U0.3 IR Proximity", False, "Respon logika LOW tidak terdeteksi", [
                "Periksa pin FC-51 IR Kiri: OUT terhubung ke Pin A3.",
                "Periksa pin FC-51 IR Kanan: OUT terhubung ke Pin D11.",
                "Putar potensiometer sensitivitas pada modul FC-51 hingga LED deteksi responsif.",
            ])
            report.u03_ir_passed = False
        report.add_subtest(st_ir)
        self.reporter.on_subtest_result(st_ir.name, st_ir.passed, st_ir.details, st_ir.pin_hints)

        # U0.4: Buzzer & LED Indicators
        beep_ok = self.transport.test_buzzer(200)
        indicators_ok = beep_ok and self.observation.verify_indicators()
        if indicators_ok:
            st_ind = U0SubtestReport("U0.4 Buzzer & LED", True, "Indikator aktif normal")
            report.u04_indicators_passed = True
        else:
            st_ind = U0SubtestReport("U0.4 Buzzer & LED", False, "Buzzer/LED tidak merespons", [
                "Periksa Buzzer: Positif terhubung ke Pin A2, negatif ke GND.",
                "Periksa LED status: Anoda ke Pin D13 (dengan resistor 220Ω), katoda ke GND.",
            ])
            report.u04_indicators_passed = False
        report.add_subtest(st_ind)
        self.reporter.on_subtest_result(st_ind.name, st_ind.passed, st_ind.details, st_ind.pin_hints)

        # U0.5: Motor Driver Channels & Optical Encoders
        pulses_l1 = self.transport.spin_motors(140, 0, 400).left
        pulses_r2 = self.transport.spin_motors(0, 140, 400).right

        motors_passed = True
        motor_hints: List[str] = []
        if pulses_l1 <= 0:
            motors_passed = False
            motor_hints.append("Encoder Kiri tidak mendeteksi pulsa! Cek disk optical encoder & Pin D2 (INT0).")
        if pulses_r2 <= 0:
            motors_passed = False
            motor_hints.append("Encoder Kanan tidak mendeteksi pulsa! Cek disk optical encoder & Pin D3 (INT1).")

        left_forward, right_forward = self.observation.verify_motor_directions()
        if not left_forward or not right_forward:
            self.reporter.on_warning(
                f"Deteksi motor terbalik: Kiri={'MAJU' if left_forward else 'MUNDUR'}, "
                f"Kanan={'MAJU' if right_forward else 'MUNDUR'}. Menerapkan Software Polarity Inversion ke EEPROM..."
            )
            try:
                prof = self.transport.read_profile()
                if not left_forward:
                    prof.invert_motor_kiri = -prof.invert_motor_kiri
                if not right_forward:
                    prof.invert_motor_kanan = -prof.invert_motor_kanan

                if self.transport.write_profile(prof):
                    self.reporter.on_success(
                        f"Polaritas motor berhasil dikoreksi di EEPROM: invertL={prof.invert_motor_kiri}, invertR={prof.invert_motor_kanan}."
                    )
                    report.polarity_inversion_applied = True
                else:
                    motors_passed = False
                    motor_hints.append("Gagal menyimpan koreksi polaritas ke EEPROM.")
            except Exception as e:
                motors_passed = False
                motor_hints.append(f"Gagal memperbarui profil EEPROM: {e}")

        if motors_passed:
            st_mot = U0SubtestReport("U0.5 Motor & Encoder", True, f"Pulsa: L={pulses_l1}, R={pulses_r2}")
            report.u05_motors_passed = True
        else:
            st_mot = U0SubtestReport("U0.5 Motor & Encoder", False, "Pulsa encoder 0 atau error arah", motor_hints)
            report.u05_motors_passed = False
        report.add_subtest(st_mot)
        self.reporter.on_subtest_result(st_mot.name, st_mot.passed, st_mot.details, st_mot.pin_hints)

        # Aggregate Gate Evaluation
        report.all_passed = (report.u01_mpu_passed and
                             report.u02_sonar_passed and
                             report.u03_ir_passed and
                             report.u04_indicators_passed and
                             report.u05_motors_passed)

        self.safety_gate.update_report(report)
        self.last_u0_report = report
        self.reporter.on_hard_gate_status(report.all_passed, report)
        return report

    def run_u1(self, max_retries: int = 5) -> U1Result:
        """
        Executes Stage U1: Wheel distance and motor balance calibration.
        Iteratively tunes wheel diameter scaling and L/R motor trim multipliers until convergence.
        """
        if max_retries < 1:
            raise ValueError("max_retries harus >= 1")
        self.safety_gate.assert_motion_permitted()
        self.reporter.on_stage_start("Stage U1", "Wheel Distance & Motor Balance Calibration")
        self.reporter.on_info("Target translasi: 100.0 cm | Toleransi galat jarak <= 3.0 cm | Disparitas pulsa <= 3.0%")

        last_result: Optional[U1Result] = None

        for iteration in range(1, max_retries + 1):
            self.reporter.on_trial_progress("U1", iteration, max_retries, "Menjalankan translasi maju 100 cm...")
            telemetry = self.transport.step_u1()
            pulses_l = telemetry.pulses_l
            pulses_r = telemetry.pulses_r
            fw_dist = telemetry.firmware_dist_cm
            self.reporter.on_info(f"Pulsa tercatat: L={pulses_l}, R={pulses_r} | Jarak kalkulasi firmware: {fw_dist:.2f} cm")

            gt_dist = self.observation.measure_u1_distance(fw_dist, iteration)
            tol = check_u1_tolerances(gt_dist, pulses_l, pulses_r)
            profile = self.transport.read_profile()

            new_diam = calculate_wheel_diameter_correction(profile.diameter_roda_mm, gt_dist, fw_dist)
            trims = calculate_motor_trim_correction(
                profile.trim_motor_kiri, profile.trim_motor_kanan, pulses_l, pulses_r
            )

            status_str = "PASS" if tol.passed else ("RETRY" if iteration < max_retries else "FAIL")
            notes = f"Galat={tol.dist_error_cm:.2f}cm, Drift={tol.pulse_drift_pct:.2f}%"

            self.reporter.on_trial_result(
                TrialRecord(
                    test_id="U1",
                    iteration=iteration,
                    status=status_str,
                    target=100.0,
                    measured=fw_dist,
                    ground_truth=gt_dist,
                    error=tol.dist_error_cm,
                    profile=profile,
                    notes=notes,
                )
            )

            last_result = U1Result(
                passed=tol.passed,
                iteration=iteration,
                ground_truth_cm=gt_dist,
                firmware_dist_cm=fw_dist,
                pulses_l=pulses_l,
                pulses_r=pulses_r,
                dist_error_cm=tol.dist_error_cm,
                pulse_drift_pct=tol.pulse_drift_pct,
                new_diameter_mm=new_diam,
                new_trim_l=trims.left,
                new_trim_r=trims.right,
            )

            # Update and persist tuned parameters
            profile.diameter_roda_mm = new_diam
            profile.trim_motor_kiri = trims.left
            profile.trim_motor_kanan = trims.right
            self.transport.write_profile(profile)

            if tol.passed:
                self.reporter.on_success(
                    f"U1 Toleransi Terpenuhi! Galat={tol.dist_error_cm:.2f} cm (<= 3.0 cm), Drift={tol.pulse_drift_pct:.2f}% (<= 3.0%). "
                    f"Profil EEPROM diperbarui: Diameter={new_diam:.2f} mm, TrimL={trims.left:.3f}, TrimR={trims.right:.3f}."
                )
                return last_result

            self.reporter.on_warning(
                f"U1 Belum Konvergen: Galat={tol.dist_error_cm:.2f} cm, Drift={tol.pulse_drift_pct:.2f}%. "
                f"Penyetelan: Diameter -> {new_diam:.2f} mm, Trim L -> {trims.left:.3f}, Trim R -> {trims.right:.3f}."
            )
            if iteration < max_retries:
                self.observation.pause_for_u1_retry(iteration)

        self.reporter.on_error(f"U1 GAGAL setelah {max_retries} iterasi! Periksa roda selip atau alignment disk encoder.")
        assert last_result is not None
        return last_result

    def run_u2(self, target_angle: float = 90.0) -> U2Result:
        """
        Executes Stage U2: Gyroscope bias sampling & in-place rotation accuracy test.
        """
        self.safety_gate.assert_motion_permitted()
        self.reporter.on_stage_start("Stage U2", "Gyroscope Bias & In-Place Rotation")
        self.reporter.on_info(f"Target rotasi in-place: {target_angle:.1f}° | Toleransi sudut <= 5.0°")

        self.reporter.on_info("[U2.1] Pengambilan Sampel Bias Gyroscope Z-axis (2.0 detik)...")
        self.observation.pause_for_u2_bias_sampling()
        self.transport.wait_for_gyro_bias(2.0)
        self.reporter.on_success("Sampling bias selesai.")

        self.reporter.on_info(f"[U2.2] Menjalankan Rotasi In-Place ({target_angle:.1f}°)...")
        reported_theta = self.transport.step_u2(target_angle)
        self.reporter.on_info(f"Sudut orientasi akhir firmware: {reported_theta:.2f}°")

        actual_angle, polarity_ok = self.observation.measure_u2_rotation(target_angle, reported_theta)
        if not polarity_ok:
            self.reporter.on_warning("Arah rotasi terbalik! Periksa orientasi pin MPU6050 atau GYRO_SIGN.")

        tol = check_u2_tolerances(actual_angle, target_angle)
        passed = tol.passed and polarity_ok

        profile = self.transport.read_profile()
        status_str = "PASS" if passed else "FAIL"
        notes = f"Target={target_angle:.1f}°, Aktual={actual_angle:.1f}°, Galat={tol.error:.2f}°"

        self.reporter.on_trial_result(
            TrialRecord(
                test_id="U2",
                iteration=1,
                status=status_str,
                target=target_angle,
                measured=reported_theta,
                ground_truth=actual_angle,
                error=tol.error,
                profile=profile,
                notes=notes,
            )
        )

        if passed:
            self.reporter.on_success(f"U2 Toleransi Terpenuhi! Galat Sudut={tol.error:.2f}° (<= 5.0°).")
        else:
            self.reporter.on_error(f"U2 Gagal Memenuhi Toleransi! Galat Sudut={tol.error:.2f}° (> 5.0°).")

        return U2Result(passed, target_angle, actual_angle, tol.error, polarity_ok)

    def run_u3(self) -> U3Result:
        """
        Executes Stage U3: Gyro-guided straight line tracking (150 cm).
        """
        self.safety_gate.assert_motion_permitted()
        self.reporter.on_stage_start("Stage U3", "Gyro-Guided Straight Line Tracking")
        self.reporter.on_info("Target translasi lurus: 150.0 cm | Toleransi deviasi lateral sumbu Y <= 5.0 cm")

        telemetry = self.transport.step_u3()
        self.reporter.on_info(
            f"Pulsa tercatat: L={telemetry.pulses_l}, R={telemetry.pulses_r} | "
            f"Maksimum galat heading: {telemetry.max_heading_err_deg:.2f}°"
        )

        lat_dev = self.observation.measure_u3_lateral_deviation(telemetry.max_heading_err_deg)
        tol = check_u3_tolerances(lat_dev)

        profile = self.transport.read_profile()
        status_str = "PASS" if tol.passed else "FAIL"
        notes = f"MaxHeadingErr={telemetry.max_heading_err_deg:.2f}°, DeviasiLateral={lat_dev:.2f}cm"

        self.reporter.on_trial_result(
            TrialRecord(
                test_id="U3",
                iteration=1,
                status=status_str,
                target=0.0,
                measured=telemetry.max_heading_err_deg,
                ground_truth=lat_dev,
                error=tol.error,
                profile=profile,
                notes=notes,
            )
        )

        if tol.passed:
            self.reporter.on_success(f"U3 Toleransi Terpenuhi! Deviasi Lateral={tol.error:.2f} cm (<= 5.0 cm).")
        else:
            self.reporter.on_error(f"U3 Gagal Memenuhi Toleransi! Deviasi Lateral={tol.error:.2f} cm (> 5.0 cm).")

        return U3Result(tol.passed, telemetry.pulses_l, telemetry.pulses_r, telemetry.max_heading_err_deg, tol.error)

    def run_u4(self) -> U4Result:
        """
        Executes Stage U4: Ultrasonic obstacle ranging accuracy test (10, 20, 40 cm).
        """
        self.safety_gate.assert_motion_permitted()
        self.reporter.on_stage_start("Stage U4", "Ultrasonic Obstacle Ranging Accuracy")
        self.reporter.on_info("Titik uji jarak: 10.0 cm, 20.0 cm, 40.0 cm | Toleransi galat jarak <= 1.0 cm")

        targets = [10.0, 20.0, 40.0]
        sub_results: List[U4SubResult] = []
        profile = self.transport.read_profile()

        for idx, target in enumerate(targets, 1):
            self.reporter.on_info(f"[U4.{idx}] Pengujian Jarak Rintangan {target:.1f} cm...")
            self.observation.prepare_u4_target(target)

            measured = self.transport.step_u4()
            tol = check_u4_tolerances(measured, target)
            sub_results.append(U4SubResult(target, measured, tol.error, tol.passed))

            status_str = "PASS" if tol.passed else "FAIL"
            notes = f"Target={target:.1f}cm, Terbaca={measured:.2f}cm, Galat={tol.error:.2f}cm"

            self.reporter.on_trial_result(
                TrialRecord(
                    test_id=f"U4.{idx}",
                    iteration=1,
                    status=status_str,
                    target=target,
                    measured=measured,
                    ground_truth=target,
                    error=tol.error,
                    profile=profile,
                    notes=notes,
                )
            )

            if tol.passed:
                self.reporter.on_success(f"U4.{idx} Terbaca: {measured:.2f} cm | Galat={tol.error:.2f} cm (<= 1.0 cm).")
            else:
                self.reporter.on_error(f"U4.{idx} Terbaca: {measured:.2f} cm | Galat={tol.error:.2f} cm (> 1.0 cm)!")

        all_passed = all(sr.passed for sr in sub_results)
        if all_passed:
            self.reporter.on_success("Seluruh titik uji ultrasonik U4 lulus kalibrasi akurasi.")
        else:
            self.reporter.on_error("U4 belum memenuhi toleransi akurasi pada sebagian titik.")

        return U4Result(all_passed, sub_results)

    def calibrate(self, stage: str = "ALL", max_u1_retries: int = 5) -> CalibrationReport:
        """
        High-level orchestration entrypoint for running calibration suites or individual stages.
        Strictly enforces the Stage U0 safety gate before proceeding with motion.
        """
        self.safety_gate.assert_motion_permitted()

        u1_res: Optional[U1Result] = None
        u2_res: Optional[U2Result] = None
        u3_res: Optional[U3Result] = None
        u4_res: Optional[U4Result] = None

        target_stage = stage.upper()
        if target_stage in ("U1", "ALL"):
            u1_res = self.run_u1(max_retries=max_u1_retries)
        if target_stage in ("U2", "ALL"):
            u2_res = self.run_u2()
        if target_stage in ("U3", "ALL"):
            u3_res = self.run_u3()
        if target_stage in ("U4", "ALL"):
            u4_res = self.run_u4()

        rep = CalibrationReport(
            u0=self.last_u0_report,
            u1=u1_res,
            u2=u2_res,
            u3=u3_res,
            u4=u4_res,
        )

        if target_stage == "ALL":
            summary_lines = [
                f"U1 Pulsa & Diameter Roda : {'LULUS' if u1_res and u1_res.passed else 'GAGAL'} (Galat={u1_res.dist_error_cm if u1_res else 0:.2f} cm)",
                f"U2 Rotasi Gyro In-Place  : {'LULUS' if u2_res and u2_res.passed else 'GAGAL'} (Galat={u2_res.angular_error_deg if u2_res else 0:.2f}°)",
                f"U3 Lintasan Lurus 150 cm : {'LULUS' if u3_res and u3_res.passed else 'GAGAL'} (Deviasi={u3_res.lateral_error_cm if u3_res else 0:.2f} cm)",
                f"U4 Akurasi Sonar HC-SR04 : {'LULUS' if u4_res and u4_res.passed else 'GAGAL'}",
            ]
            self.reporter.on_stage_summary("Ringkasan Status Kalibrasi Kinematika", rep.all_passed, summary_lines)

        return rep
