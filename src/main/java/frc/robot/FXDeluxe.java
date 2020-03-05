/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseMotorControllerConfiguration;
import com.ctre.phoenix.motorcontrol.can.BasePIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

/**
 * Add your docs here.
 */
public class FXDeluxe extends TalonFX {
    public FXDeluxe() {
        super(1);
    }


    @Override
    public ErrorCode configAllSettings(TalonFXConfiguration allConfigs) {
        // TODO Auto-generated method stub
        return super.configAllSettings(allConfigs);
    }

    @Override
    public ErrorCode configAllSettings(TalonFXConfiguration allConfigs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configAllSettings(allConfigs, timeoutMs);
    }

    @Override
    public MotorCommutation configGetMotorCommutation() {
        // TODO Auto-generated method stub
        return super.configGetMotorCommutation();
    }

    @Override
    public MotorCommutation configGetMotorCommutation(int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configGetMotorCommutation(timeoutMs);
    }

    @Override
    public ErrorCode configGetStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitConfigsToFill) {
        // TODO Auto-generated method stub
        return super.configGetStatorCurrentLimit(currLimitConfigsToFill);
    }

    @Override
    public ErrorCode configGetStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitConfigsToFill,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configGetStatorCurrentLimit(currLimitConfigsToFill, timeoutMs);
    }

    @Override
    public ErrorCode configGetSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitConfigsToFill) {
        // TODO Auto-generated method stub
        return super.configGetSupplyCurrentLimit(currLimitConfigsToFill);
    }

    @Override
    public ErrorCode configGetSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitConfigsToFill,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configGetSupplyCurrentLimit(currLimitConfigsToFill, timeoutMs);
    }

    @Override
    public ErrorCode configIntegratedSensorAbsoluteRange(AbsoluteSensorRange absoluteSensorRange) {
        // TODO Auto-generated method stub
        return super.configIntegratedSensorAbsoluteRange(absoluteSensorRange);
    }

    @Override
    public ErrorCode configIntegratedSensorAbsoluteRange(AbsoluteSensorRange absoluteSensorRange, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configIntegratedSensorAbsoluteRange(absoluteSensorRange, timeoutMs);
    }

    @Override
    public ErrorCode configIntegratedSensorInitializationStrategy(SensorInitializationStrategy initializationStrategy) {
        // TODO Auto-generated method stub
        return super.configIntegratedSensorInitializationStrategy(initializationStrategy);
    }

    @Override
    public ErrorCode configIntegratedSensorInitializationStrategy(SensorInitializationStrategy initializationStrategy,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configIntegratedSensorInitializationStrategy(initializationStrategy, timeoutMs);
    }

    @Override
    public ErrorCode configIntegratedSensorOffset(double offsetDegrees) {
        // TODO Auto-generated method stub
        return super.configIntegratedSensorOffset(offsetDegrees);
    }

    @Override
    public ErrorCode configIntegratedSensorOffset(double offsetDegrees, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configIntegratedSensorOffset(offsetDegrees, timeoutMs);
    }

    @Override
    public ErrorCode configMotorCommutation(MotorCommutation motorCommutation) {
        // TODO Auto-generated method stub
        return super.configMotorCommutation(motorCommutation);
    }

    @Override
    public ErrorCode configMotorCommutation(MotorCommutation motorCommutation, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configMotorCommutation(motorCommutation, timeoutMs);
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(TalonFXFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
    }

    @Override
    public ErrorCode configStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitCfg) {
        // TODO Auto-generated method stub
        return super.configStatorCurrentLimit(currLimitCfg);
    }

    @Override
    public ErrorCode configStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configStatorCurrentLimit(currLimitCfg, timeoutMs);
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg) {
        // TODO Auto-generated method stub
        return super.configSupplyCurrentLimit(currLimitCfg);
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSupplyCurrentLimit(currLimitCfg, timeoutMs);
    }

    @Override
    protected ErrorCode configurePID(TalonFXPIDSetConfiguration pid) {
        // TODO Auto-generated method stub
        return super.configurePID(pid);
    }

    @Override
    protected ErrorCode configurePID(TalonFXPIDSetConfiguration pid, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configurePID(pid, pidIdx, timeoutMs);
    }

    @Override
    public ErrorCode getAllConfigs(TalonFXConfiguration allConfigs) {
        // TODO Auto-generated method stub
        return super.getAllConfigs(allConfigs);
    }

    @Override
    public ErrorCode getAllConfigs(TalonFXConfiguration allConfigs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.getAllConfigs(allConfigs, timeoutMs);
    }

    @Override
    public void getPIDConfigs(TalonFXPIDSetConfiguration pid) {
        // TODO Auto-generated method stub
        super.getPIDConfigs(pid);
    }

    @Override
    public void getPIDConfigs(TalonFXPIDSetConfiguration pid, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        super.getPIDConfigs(pid, pidIdx, timeoutMs);
    }

    @Override
    public TalonFXSensorCollection getSensorCollection() {
        // TODO Auto-generated method stub
        return super.getSensorCollection();
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        // TODO Auto-generated method stub
        super.set(mode, value);
    }

    @Override
    public void set(TalonFXControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        // TODO Auto-generated method stub
        super.set(mode, demand0, demand1Type, demand1);
    }

    @Override
    public void setInverted(TalonFXInvertType invertType) {
        // TODO Auto-generated method stub
        super.setInverted(invertType);
    }

    @Override
    protected ErrorCode configAllSettings(BaseTalonConfiguration allConfigs) {
        // TODO Auto-generated method stub
        return super.configAllSettings(allConfigs);
    }

    @Override
    protected ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configAllSettings(allConfigs, timeoutMs);
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        // TODO Auto-generated method stub
        return super.configForwardLimitSwitchSource(type, normalOpenOrClose);
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configForwardLimitSwitchSource(type, normalOpenOrClose, timeoutMs);
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
        // TODO Auto-generated method stub
        return super.configReverseLimitSwitchSource(type, normalOpenOrClose);
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configReverseLimitSwitchSource(type, normalOpenOrClose, timeoutMs);
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period) {
        // TODO Auto-generated method stub
        return super.configVelocityMeasurementPeriod(period);
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configVelocityMeasurementPeriod(period, timeoutMs);
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize) {
        // TODO Auto-generated method stub
        return super.configVelocityMeasurementWindow(windowSize);
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configVelocityMeasurementWindow(windowSize, timeoutMs);
    }

    @Override
    protected ErrorCode configurePID(BaseTalonPIDSetConfiguration pid) {
        // TODO Auto-generated method stub
        return super.configurePID(pid);
    }

    @Override
    protected ErrorCode configurePID(BaseTalonPIDSetConfiguration pid, int pidIdx, int timeoutMs,
            boolean enableOptimizations) {
        // TODO Auto-generated method stub
        return super.configurePID(pid, pidIdx, timeoutMs, enableOptimizations);
    }

    @Override
    protected void getAllConfigs(BaseTalonConfiguration allConfigs) {
        // TODO Auto-generated method stub
        super.getAllConfigs(allConfigs);
    }

    @Override
    protected void getAllConfigs(BaseTalonConfiguration allConfigs, int timeoutMs) {
        // TODO Auto-generated method stub
        super.getAllConfigs(allConfigs, timeoutMs);
    }

    @Override
    public double getOutputCurrent() {
        // TODO Auto-generated method stub
        return super.getOutputCurrent();
    }

    @Override
    protected void getPIDConfigs(BaseTalonPIDSetConfiguration pid) {
        // TODO Auto-generated method stub
        super.getPIDConfigs(pid);
    }

    @Override
    protected void getPIDConfigs(BaseTalonPIDSetConfiguration pid, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        super.getPIDConfigs(pid, pidIdx, timeoutMs);
    }

    @Override
    public double getStatorCurrent() {
        // TODO Auto-generated method stub
        return super.getStatorCurrent();
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame) {
        // TODO Auto-generated method stub
        return super.getStatusFramePeriod(frame);
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.getStatusFramePeriod(frame, timeoutMs);
    }

    @Override
    public double getSupplyCurrent() {
        // TODO Auto-generated method stub
        return super.getSupplyCurrent();
    }

    @Override
    protected TalonFXSensorCollection getTalonFXSensorCollection() {
        // TODO Auto-generated method stub
        return super.getTalonFXSensorCollection();
    }

    @Override
    protected SensorCollection getTalonSRXSensorCollection() {
        // TODO Auto-generated method stub
        return super.getTalonSRXSensorCollection();
    }

    @Override
    public int isFwdLimitSwitchClosed() {
        // TODO Auto-generated method stub
        return super.isFwdLimitSwitchClosed();
    }

    @Override
    public int isRevLimitSwitchClosed() {
        // TODO Auto-generated method stub
        return super.isRevLimitSwitchClosed();
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
        // TODO Auto-generated method stub
        return super.setStatusFramePeriod(frame, periodMs);
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.setStatusFramePeriod(frame, periodMs, timeoutMs);
    }

    @Override
    public ErrorCode DestroyObject() {
        // TODO Auto-generated method stub
        return super.DestroyObject();
    }

    @Override
    protected ErrorCode baseConfigAllSettings(BaseMotorControllerConfiguration allConfigs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.baseConfigAllSettings(allConfigs, timeoutMs);
    }

    @Override
    protected ErrorCode baseConfigurePID(BasePIDSetConfiguration pid, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.baseConfigurePID(pid, pidIdx, timeoutMs);
    }

    @Override
    protected void baseGetAllConfigs(BaseMotorControllerConfiguration allConfigs, int timeoutMs) {
        // TODO Auto-generated method stub
        super.baseGetAllConfigs(allConfigs, timeoutMs);
    }

    @Override
    protected void baseGetPIDConfigs(BasePIDSetConfiguration pid, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        super.baseGetPIDConfigs(pid, pidIdx, timeoutMs);
    }

    @Override
    public ErrorCode changeMotionControlFramePeriod(int periodMs) {
        // TODO Auto-generated method stub
        return super.changeMotionControlFramePeriod(periodMs);
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun() {
        // TODO Auto-generated method stub
        return super.clearMotionProfileHasUnderrun();
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
        // TODO Auto-generated method stub
        return super.clearMotionProfileHasUnderrun(timeoutMs);
    }

    @Override
    public ErrorCode clearMotionProfileTrajectories() {
        // TODO Auto-generated method stub
        return super.clearMotionProfileTrajectories();
    }

    @Override
    public ErrorCode clearStickyFaults() {
        // TODO Auto-generated method stub
        return super.clearStickyFaults();
    }

    @Override
    public ErrorCode clearStickyFaults(int timeoutMs) {
        // TODO Auto-generated method stub
        return super.clearStickyFaults(timeoutMs);
    }

    @Override
    public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableClosedLoopError) {
        // TODO Auto-generated method stub
        return super.configAllowableClosedloopError(slotIdx, allowableClosedLoopError);
    }

    @Override
    public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableClosedLoopError, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configAllowableClosedloopError(slotIdx, allowableClosedLoopError, timeoutMs);
    }

    @Override
    public ErrorCode configAuxPIDPolarity(boolean invert) {
        // TODO Auto-generated method stub
        return super.configAuxPIDPolarity(invert);
    }

    @Override
    public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configAuxPIDPolarity(invert, timeoutMs);
    }

    @Override
    public ErrorCode configClearPositionOnLimitF(boolean clearPositionOnLimitF, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configClearPositionOnLimitF(clearPositionOnLimitF, timeoutMs);
    }

    @Override
    public ErrorCode configClearPositionOnLimitR(boolean clearPositionOnLimitR, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs);
    }

    @Override
    public ErrorCode configClearPositionOnQuadIdx(boolean clearPositionOnQuadIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configClearPositionOnQuadIdx(clearPositionOnQuadIdx, timeoutMs);
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut) {
        // TODO Auto-generated method stub
        return super.configClosedLoopPeakOutput(slotIdx, percentOut);
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configClosedLoopPeakOutput(slotIdx, percentOut, timeoutMs);
    }

    @Override
    public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs) {
        // TODO Auto-generated method stub
        return super.configClosedLoopPeriod(slotIdx, loopTimeMs);
    }

    @Override
    public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configClosedLoopPeriod(slotIdx, loopTimeMs, timeoutMs);
    }

    @Override
    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull) {
        // TODO Auto-generated method stub
        return super.configClosedloopRamp(secondsFromNeutralToFull);
    }

    @Override
    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configClosedloopRamp(secondsFromNeutralToFull, timeoutMs);
    }

    @Override
    public ErrorCode configFactoryDefault() {
        // TODO Auto-generated method stub
        return super.configFactoryDefault();
    }

    @Override
    public ErrorCode configFactoryDefault(int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configFactoryDefault(timeoutMs);
    }

    @Override
    public ErrorCode configFeedbackNotContinuous(boolean feedbackNotContinuous, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configFeedbackNotContinuous(feedbackNotContinuous, timeoutMs);
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID) {
        // TODO Auto-generated method stub
        return super.configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID);
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID, timeoutMs);
    }

    @Override
    protected ErrorCode configForwardLimitSwitchSource(int typeValue, int normalOpenOrCloseValue, int deviceID,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configForwardLimitSwitchSource(typeValue, normalOpenOrCloseValue, deviceID, timeoutMs);
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable) {
        // TODO Auto-generated method stub
        return super.configForwardSoftLimitEnable(enable);
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configForwardSoftLimitEnable(enable, timeoutMs);
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit) {
        // TODO Auto-generated method stub
        return super.configForwardSoftLimitThreshold(forwardSensorLimit);
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configForwardSoftLimitThreshold(forwardSensorLimit, timeoutMs);
    }

    @Override
    public int configGetCustomParam(int paramIndex) {
        // TODO Auto-generated method stub
        return super.configGetCustomParam(paramIndex);
    }

    @Override
    public int configGetCustomParam(int paramIndex, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configGetCustomParam(paramIndex, timeoutMs);
    }

    @Override
    public double configGetParameter(ParamEnum param, int ordinal) {
        // TODO Auto-generated method stub
        return super.configGetParameter(param, ordinal);
    }

    @Override
    public double configGetParameter(int param, int ordinal) {
        // TODO Auto-generated method stub
        return super.configGetParameter(param, ordinal);
    }

    @Override
    public double configGetParameter(ParamEnum param, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configGetParameter(param, ordinal, timeoutMs);
    }

    @Override
    public double configGetParameter(int param, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configGetParameter(param, ordinal, timeoutMs);
    }

    @Override
    public ErrorCode configLimitSwitchDisableNeutralOnLOS(boolean limitSwitchDisableNeutralOnLOS, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configLimitSwitchDisableNeutralOnLOS(limitSwitchDisableNeutralOnLOS, timeoutMs);
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum) {
        // TODO Auto-generated method stub
        return super.configMaxIntegralAccumulator(slotIdx, iaccum);
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configMaxIntegralAccumulator(slotIdx, iaccum, timeoutMs);
    }

    @Override
    public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec) {
        // TODO Auto-generated method stub
        return super.configMotionAcceleration(sensorUnitsPer100msPerSec);
    }

    @Override
    public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configMotionAcceleration(sensorUnitsPer100msPerSec, timeoutMs);
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms) {
        // TODO Auto-generated method stub
        return super.configMotionCruiseVelocity(sensorUnitsPer100ms);
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryInterpolationEnable(boolean enable) {
        // TODO Auto-generated method stub
        return super.configMotionProfileTrajectoryInterpolationEnable(enable);
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryInterpolationEnable(boolean enable, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configMotionProfileTrajectoryInterpolationEnable(enable, timeoutMs);
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs) {
        // TODO Auto-generated method stub
        return super.configMotionProfileTrajectoryPeriod(baseTrajDurationMs);
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configMotionProfileTrajectoryPeriod(baseTrajDurationMs, timeoutMs);
    }

    @Override
    public ErrorCode configMotionSCurveStrength(int curveStrength) {
        // TODO Auto-generated method stub
        return super.configMotionSCurveStrength(curveStrength);
    }

    @Override
    public ErrorCode configMotionSCurveStrength(int curveStrength, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configMotionSCurveStrength(curveStrength, timeoutMs);
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband) {
        // TODO Auto-generated method stub
        return super.configNeutralDeadband(percentDeadband);
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configNeutralDeadband(percentDeadband, timeoutMs);
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut) {
        // TODO Auto-generated method stub
        return super.configNominalOutputForward(percentOut);
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configNominalOutputForward(percentOut, timeoutMs);
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut) {
        // TODO Auto-generated method stub
        return super.configNominalOutputReverse(percentOut);
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configNominalOutputReverse(percentOut, timeoutMs);
    }

    @Override
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull) {
        // TODO Auto-generated method stub
        return super.configOpenloopRamp(secondsFromNeutralToFull);
    }

    @Override
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configOpenloopRamp(secondsFromNeutralToFull, timeoutMs);
    }

    @Override
    public ErrorCode configPeakOutputForward(double percentOut) {
        // TODO Auto-generated method stub
        return super.configPeakOutputForward(percentOut);
    }

    @Override
    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configPeakOutputForward(percentOut, timeoutMs);
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut) {
        // TODO Auto-generated method stub
        return super.configPeakOutputReverse(percentOut);
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configPeakOutputReverse(percentOut, timeoutMs);
    }

    @Override
    public ErrorCode configPulseWidthPeriod_EdgesPerRot(int pulseWidthPeriod_EdgesPerRot, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configPulseWidthPeriod_EdgesPerRot(pulseWidthPeriod_EdgesPerRot, timeoutMs);
    }

    @Override
    public ErrorCode configPulseWidthPeriod_FilterWindowSz(int pulseWidthPeriod_FilterWindowSz, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configPulseWidthPeriod_FilterWindowSz(pulseWidthPeriod_FilterWindowSz, timeoutMs);
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal) {
        // TODO Auto-generated method stub
        return super.configRemoteFeedbackFilter(canCoderRef, remoteOrdinal);
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource,
            int remoteOrdinal) {
        // TODO Auto-generated method stub
        return super.configRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal);
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configRemoteFeedbackFilter(canCoderRef, remoteOrdinal, timeoutMs);
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal, timeoutMs);
    }

    @Override
    public ErrorCode configRemoteSensorClosedLoopDisableNeutralOnLOS(boolean remoteSensorClosedLoopDisableNeutralOnLOS,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configRemoteSensorClosedLoopDisableNeutralOnLOS(remoteSensorClosedLoopDisableNeutralOnLOS,
                timeoutMs);
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID) {
        // TODO Auto-generated method stub
        return super.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID);
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID, timeoutMs);
    }

    @Override
    protected ErrorCode configReverseLimitSwitchSource(int typeValue, int normalOpenOrCloseValue, int deviceID,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configReverseLimitSwitchSource(typeValue, normalOpenOrCloseValue, deviceID, timeoutMs);
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable) {
        // TODO Auto-generated method stub
        return super.configReverseSoftLimitEnable(enable);
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configReverseSoftLimitEnable(enable, timeoutMs);
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit) {
        // TODO Auto-generated method stub
        return super.configReverseSoftLimitThreshold(reverseSensorLimit);
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configReverseSoftLimitThreshold(reverseSensorLimit, timeoutMs);
    }

    @Override
    public ErrorCode configSelectedFeedbackCoefficient(double coefficient) {
        // TODO Auto-generated method stub
        return super.configSelectedFeedbackCoefficient(coefficient);
    }

    @Override
    public ErrorCode configSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSelectedFeedbackCoefficient(coefficient, pidIdx, timeoutMs);
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice) {
        // TODO Auto-generated method stub
        return super.configSelectedFeedbackSensor(feedbackDevice);
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice) {
        // TODO Auto-generated method stub
        return super.configSelectedFeedbackSensor(feedbackDevice);
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice) {
        // TODO Auto-generated method stub
        return super.configSensorTerm(sensorTerm, feedbackDevice);
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, RemoteFeedbackDevice feedbackDevice) {
        // TODO Auto-generated method stub
        return super.configSensorTerm(sensorTerm, feedbackDevice);
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSensorTerm(sensorTerm, feedbackDevice, timeoutMs);
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, RemoteFeedbackDevice feedbackDevice, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSensorTerm(sensorTerm, feedbackDevice, timeoutMs);
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex) {
        // TODO Auto-generated method stub
        return super.configSetCustomParam(newValue, paramIndex);
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSetCustomParam(newValue, paramIndex, timeoutMs);
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        // TODO Auto-generated method stub
        return super.configSetParameter(param, value, subValue, ordinal);
    }

    @Override
    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal) {
        // TODO Auto-generated method stub
        return super.configSetParameter(param, value, subValue, ordinal);
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSetParameter(param, value, subValue, ordinal, timeoutMs);
    }

    @Override
    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSetParameter(param, value, subValue, ordinal, timeoutMs);
    }

    @Override
    public ErrorCode configSoftLimitDisableNeutralOnLOS(boolean softLimitDisableNeutralOnLOS, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configSoftLimitDisableNeutralOnLOS(softLimitDisableNeutralOnLOS, timeoutMs);
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage) {
        // TODO Auto-generated method stub
        return super.configVoltageCompSaturation(voltage);
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configVoltageCompSaturation(voltage, timeoutMs);
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples) {
        // TODO Auto-generated method stub
        return super.configVoltageMeasurementFilter(filterWindowSamples);
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configVoltageMeasurementFilter(filterWindowSamples, timeoutMs);
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIdx, int izone) {
        // TODO Auto-generated method stub
        return super.config_IntegralZone(slotIdx, izone);
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIdx, int izone, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.config_IntegralZone(slotIdx, izone, timeoutMs);
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value) {
        // TODO Auto-generated method stub
        return super.config_kD(slotIdx, value);
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.config_kD(slotIdx, value, timeoutMs);
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value) {
        // TODO Auto-generated method stub
        return super.config_kF(slotIdx, value);
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.config_kF(slotIdx, value, timeoutMs);
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value) {
        // TODO Auto-generated method stub
        return super.config_kI(slotIdx, value);
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.config_kI(slotIdx, value, timeoutMs);
    }

    @Override
    public ErrorCode config_kP(int slotIdx, double value) {
        // TODO Auto-generated method stub
        return super.config_kP(slotIdx, value);
    }

    @Override
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.config_kP(slotIdx, value, timeoutMs);
    }

    @Override
    public ErrorCode configureFilter(FilterConfiguration filter) {
        // TODO Auto-generated method stub
        return super.configureFilter(filter);
    }

    @Override
    public ErrorCode configureFilter(FilterConfiguration filter, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configureFilter(filter, ordinal, timeoutMs);
    }

    @Override
    public ErrorCode configureFilter(FilterConfiguration filter, int ordinal, int timeoutMs,
            boolean enableOptimizations) {
        // TODO Auto-generated method stub
        return super.configureFilter(filter, ordinal, timeoutMs, enableOptimizations);
    }

    @Override
    public ErrorCode configureSlot(SlotConfiguration slot) {
        // TODO Auto-generated method stub
        return super.configureSlot(slot);
    }

    @Override
    public ErrorCode configureSlot(SlotConfiguration slot, int slotIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.configureSlot(slot, slotIdx, timeoutMs);
    }

    @Override
    public void enableHeadingHold(boolean enable) {
        // TODO Auto-generated method stub
        super.enableHeadingHold(enable);
    }

    @Override
    public void enableVoltageCompensation(boolean enable) {
        // TODO Auto-generated method stub
        super.enableVoltageCompensation(enable);
    }

    @Override
    public void follow(IMotorController masterToFollow) {
        // TODO Auto-generated method stub
        super.follow(masterToFollow);
    }

    @Override
    public void follow(IMotorController masterToFollow, FollowerType followerType) {
        // TODO Auto-generated method stub
        super.follow(masterToFollow, followerType);
    }

    @Override
    public double getActiveTrajectoryArbFeedFwd() {
        // TODO Auto-generated method stub
        return super.getActiveTrajectoryArbFeedFwd();
    }

    @Override
    public double getActiveTrajectoryArbFeedFwd(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getActiveTrajectoryArbFeedFwd(pidIdx);
    }

    @Override
    public double getActiveTrajectoryHeading() {
        // TODO Auto-generated method stub
        return super.getActiveTrajectoryHeading();
    }

    @Override
    public int getActiveTrajectoryPosition() {
        // TODO Auto-generated method stub
        return super.getActiveTrajectoryPosition();
    }

    @Override
    public int getActiveTrajectoryPosition(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getActiveTrajectoryPosition(pidIdx);
    }

    @Override
    public int getActiveTrajectoryVelocity() {
        // TODO Auto-generated method stub
        return super.getActiveTrajectoryVelocity();
    }

    @Override
    public int getActiveTrajectoryVelocity(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getActiveTrajectoryVelocity(pidIdx);
    }

    @Override
    public int getBaseID() {
        // TODO Auto-generated method stub
        return super.getBaseID();
    }

    @Override
    public double getBusVoltage() {
        // TODO Auto-generated method stub
        return super.getBusVoltage();
    }

    @Override
    public int getClosedLoopError() {
        // TODO Auto-generated method stub
        return super.getClosedLoopError();
    }

    @Override
    public int getClosedLoopError(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getClosedLoopError(pidIdx);
    }

    @Override
    public double getClosedLoopTarget() {
        // TODO Auto-generated method stub
        return super.getClosedLoopTarget();
    }

    @Override
    public double getClosedLoopTarget(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getClosedLoopTarget(pidIdx);
    }

    @Override
    public ControlMode getControlMode() {
        // TODO Auto-generated method stub
        return super.getControlMode();
    }

    @Override
    public int getDeviceID() {
        // TODO Auto-generated method stub
        return super.getDeviceID();
    }

    @Override
    public double getErrorDerivative() {
        // TODO Auto-generated method stub
        return super.getErrorDerivative();
    }

    @Override
    public double getErrorDerivative(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getErrorDerivative(pidIdx);
    }

    @Override
    public ErrorCode getFaults(Faults toFill) {
        // TODO Auto-generated method stub
        return super.getFaults(toFill);
    }

    @Override
    public void getFilterConfigs(FilterConfiguration filter) {
        // TODO Auto-generated method stub
        super.getFilterConfigs(filter);
    }

    @Override
    public void getFilterConfigs(FilterConfiguration filter, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        super.getFilterConfigs(filter, ordinal, timeoutMs);
    }

    @Override
    public int getFirmwareVersion() {
        // TODO Auto-generated method stub
        return super.getFirmwareVersion();
    }

    @Override
    public long getHandle() {
        // TODO Auto-generated method stub
        return super.getHandle();
    }

    @Override
    public double getIntegralAccumulator() {
        // TODO Auto-generated method stub
        return super.getIntegralAccumulator();
    }

    @Override
    public double getIntegralAccumulator(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getIntegralAccumulator(pidIdx);
    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        return super.getInverted();
    }

    @Override
    public ErrorCode getLastError() {
        // TODO Auto-generated method stub
        return super.getLastError();
    }

    @Override
    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
        // TODO Auto-generated method stub
        return super.getMotionProfileStatus(statusToFill);
    }

    @Override
    public int getMotionProfileTopLevelBufferCount() {
        // TODO Auto-generated method stub
        return super.getMotionProfileTopLevelBufferCount();
    }

    @Override
    public double getMotorOutputPercent() {
        // TODO Auto-generated method stub
        return super.getMotorOutputPercent();
    }

    @Override
    public double getMotorOutputVoltage() {
        // TODO Auto-generated method stub
        return super.getMotorOutputVoltage();
    }

    @Override
    public int getSelectedSensorPosition() {
        // TODO Auto-generated method stub
        return super.getSelectedSensorPosition();
    }

    @Override
    public int getSelectedSensorPosition(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getSelectedSensorPosition(pidIdx);
    }

    @Override
    public int getSelectedSensorVelocity() {
        // TODO Auto-generated method stub
        return super.getSelectedSensorVelocity();
    }

    @Override
    public int getSelectedSensorVelocity(int pidIdx) {
        // TODO Auto-generated method stub
        return super.getSelectedSensorVelocity(pidIdx);
    }

    @Override
    public void getSlotConfigs(SlotConfiguration slot) {
        // TODO Auto-generated method stub
        super.getSlotConfigs(slot);
    }

    @Override
    public void getSlotConfigs(SlotConfiguration slot, int slotIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        super.getSlotConfigs(slot, slotIdx, timeoutMs);
    }

    @Override
    public int getStatusFramePeriod(int frame) {
        // TODO Auto-generated method stub
        return super.getStatusFramePeriod(frame);
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame) {
        // TODO Auto-generated method stub
        return super.getStatusFramePeriod(frame);
    }

    @Override
    public int getStatusFramePeriod(int frame, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.getStatusFramePeriod(frame, timeoutMs);
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.getStatusFramePeriod(frame, timeoutMs);
    }

    @Override
    public ErrorCode getStickyFaults(StickyFaults toFill) {
        // TODO Auto-generated method stub
        return super.getStickyFaults(toFill);
    }

    @Override
    public double getTemperature() {
        // TODO Auto-generated method stub
        return super.getTemperature();
    }

    @Override
    public boolean hasResetOccurred() {
        // TODO Auto-generated method stub
        return super.hasResetOccurred();
    }

    @Override
    public boolean isMotionProfileFinished() {
        // TODO Auto-generated method stub
        return super.isMotionProfileFinished();
    }

    @Override
    public boolean isMotionProfileTopLevelBufferFull() {
        // TODO Auto-generated method stub
        return super.isMotionProfileTopLevelBufferFull();
    }

    @Override
    public boolean isVoltageCompensationEnabled() {
        // TODO Auto-generated method stub
        return super.isVoltageCompensationEnabled();
    }

    @Override
    public void neutralOutput() {
        // TODO Auto-generated method stub
        super.neutralOutput();
    }

    @Override
    public void overrideLimitSwitchesEnable(boolean enable) {
        // TODO Auto-generated method stub
        super.overrideLimitSwitchesEnable(enable);
    }

    @Override
    public void overrideSoftLimitsEnable(boolean enable) {
        // TODO Auto-generated method stub
        super.overrideSoftLimitsEnable(enable);
    }

    @Override
    public void processMotionProfileBuffer() {
        // TODO Auto-generated method stub
        super.processMotionProfileBuffer();
    }

    @Override
    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        // TODO Auto-generated method stub
        return super.pushMotionProfileTrajectory(trajPt);
    }

    @Override
    public void selectDemandType(boolean value) {
        // TODO Auto-generated method stub
        super.selectDemandType(value);
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
        // TODO Auto-generated method stub
        super.selectProfileSlot(slotIdx, pidIdx);
    }

    @Override
    public void set(ControlMode mode, double outputValue) {
        // TODO Auto-generated method stub
        super.set(mode, outputValue);
    }

    @Override
    public void set(ControlMode mode, double demand0, double demand1) {
        // TODO Auto-generated method stub
        super.set(mode, demand0, demand1);
    }

    @Override
    public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        // TODO Auto-generated method stub
        super.set(mode, demand0, demand1Type, demand1);
    }

    @Override
    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
        // TODO Auto-generated method stub
        return super.setControlFramePeriod(frame, periodMs);
    }

    @Override
    public ErrorCode setControlFramePeriod(int frame, int periodMs) {
        // TODO Auto-generated method stub
        return super.setControlFramePeriod(frame, periodMs);
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum) {
        // TODO Auto-generated method stub
        return super.setIntegralAccumulator(iaccum);
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.setIntegralAccumulator(iaccum, pidIdx, timeoutMs);
    }

    @Override
    public void setInverted(boolean invert) {
        // TODO Auto-generated method stub
        super.setInverted(invert);
    }

    @Override
    public void setInverted(InvertType invertType) {
        // TODO Auto-generated method stub
        super.setInverted(invertType);
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        // TODO Auto-generated method stub
        super.setNeutralMode(neutralMode);
    }

    @Override
    public ErrorCode setSelectedSensorPosition(int sensorPos) {
        // TODO Auto-generated method stub
        return super.setSelectedSensorPosition(sensorPos);
    }

    @Override
    public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.setSelectedSensorPosition(sensorPos, pidIdx, timeoutMs);
    }

    @Override
    public void setSensorPhase(boolean PhaseSensor) {
        // TODO Auto-generated method stub
        super.setSensorPhase(PhaseSensor);
    }

    @Override
    public ErrorCode setStatusFramePeriod(int frameValue, int periodMs) {
        // TODO Auto-generated method stub
        return super.setStatusFramePeriod(frameValue, periodMs);
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs) {
        // TODO Auto-generated method stub
        return super.setStatusFramePeriod(frame, periodMs);
    }

    @Override
    public ErrorCode setStatusFramePeriod(int frameValue, int periodMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.setStatusFramePeriod(frameValue, periodMs, timeoutMs);
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return super.setStatusFramePeriod(frame, periodMs, timeoutMs);
    }

    @Override
    public ErrorCode startMotionProfile(BufferedTrajectoryPointStream stream, int minBufferedPts,
            ControlMode motionProfControlMode) {
        // TODO Auto-generated method stub
        return super.startMotionProfile(stream, minBufferedPts, motionProfControlMode);
    }

    @Override
    public void valueUpdated() {
        // TODO Auto-generated method stub
        super.valueUpdated();
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        // TODO Auto-generated method stub
        return super.clone();
    }

    @Override
    public boolean equals(Object obj) {
        // TODO Auto-generated method stub
        return super.equals(obj);
    }

    @Override
    protected void finalize() throws Throwable {
        // TODO Auto-generated method stub
        super.finalize();
    }

    @Override
    public int hashCode() {
        // TODO Auto-generated method stub
        return super.hashCode();
    }

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return super.toString();
    }
}
