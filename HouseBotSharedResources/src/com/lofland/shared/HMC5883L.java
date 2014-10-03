package com.lofland.shared;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;


//import nl.totan.sensors.MagnetoData.AngleUnits;
//import nl.totan.sensors.MagnetoData.MagnetoUnits;

import com.lofland.shared.HMC5883L.Samples.AngleUnits;
import com.lofland.shared.HMC5883L.Samples.MagnetoUnits;

import lejos.nxt.I2CPort;
import lejos.nxt.I2CSensor;
import lejos.robotics.DirectionFinder;
import lejos.util.Delay;
import lejos.util.EndianTools;

/**
 * Driver for the Dexter Industries compass sensor
 * 
 * @author Aswin
 * @version 1.0
 * 
 */
public class HMC5883L extends I2CSensor implements /*MagnetoData,*/ DirectionFinder {

	// sensor configuration
	SampleRate									sampleRate					= SampleRate._15Hz;
	MeasurementMode							measurementMode			= MeasurementMode.Normal;
	Range												range								= Range._1_3Ga;
	OperatingMode								operatingMode				= OperatingMode.SingleMeasurement;
	Samples											samples							= Samples._8;
	protected MagnetoUnits			magnetoUnit					= MagnetoUnits.milliGauss;
	protected AngleUnits				angleUnit						= AngleUnits.Radians;
	protected float							cartesianZero				= 0;

	// earth magnetic field properties (of current location)
	protected float							fieldStrength				= 0;
	protected float							margin							= 0.1f;
	protected float							inclination					= 0;
	protected float							declination					= 0;
	
	// sensor register adresses
	private static final int		DEFAULT_I2C_ADDRESS	= 0x3C;
	protected static final int	REG_CONFIG					= 0x00;
	protected static final int	REG_MAGNETO					= 0x03;
	protected static final int	REG_STATUS					= 0x09;																

	// local variables for common use
	int[]												raw									= new int[3];
	byte[]											buf									= new byte[6];

	// calibration
	boolean											calibrating					= false;
	float[]											min									= new float[3];
	float[]											max									= new float[3];
	public float[]											offset							= new float[3];

	/**
	 * Constructor for the driver. Also loads calibration settings when available.
	 * 
	 * @param port
	 */
	public HMC5883L(I2CPort port) {
		super(port, DEFAULT_I2C_ADDRESS, I2CPort.LEGO_MODE, I2CPort.TYPE_LOWSPEED);
		configureSensor();
		loadCalibration();
	}

	/**
	 * Sets the configuration registers of the sensor according to the current
	 * settings
	 */
	protected void configureSensor() {
		buf[0] = (byte) (samples.code() + sampleRate.code() + measurementMode.code());
		buf[1] = (byte) (range.code());
		buf[2] = (byte) (operatingMode.code());
		sendData(REG_CONFIG, buf, 3);
		// first measurement after configuration is not yet configured properly;
		fetchAllMagneto(new float[3]);
	}

	/**
	 * Fills an array of floats with measurements from the sensor in the default
	 * unit.
	 * <p>
	 * The array order is X, Y, Z
	 */
	public void fetchAllMagneto(float[] ret) {
		fetchAllMagneto(ret, magnetoUnit);
	}
	
	/**
	 * Fills an array of floats with measurements from the sensor in the specified
	 * unit.
	 * <p>
	 * The array order is X, Y, Z
	 * <P>
	 * When the sensor is idle zeros will be returned.
	 */
	public void fetchAllMagneto(float[] ret, MagnetoUnits unit) {
		float factor = 1.0f / range.gain();
		// get raw data
		switch (operatingMode) {
			case SingleMeasurement:
				fetchSingleMeasurementMode(raw);
				break;
			case Continuous:
				fetchContinuousMeasurementMode(raw);
				break;
			case Idle:
				for (int i = 0; i < 3; i++)
					ret[i] = 0;
				return;
			default:
		}

		// convert raw values to gauss using gain factor;
		for (int i = 0; i < 3; i++)
			ret[i] = (float) raw[i] * factor;

		// collect minimum and maximum values while calibrating;
		if (calibrating) {
			for (int i = 0; i < 3; i++) {
				if (ret[i] < min[i])
					min[i] = ret[i];
				if (ret[i] > max[i])
					max[i] = ret[i];
			}
		}

		// apply offset correction to get calibrated values
		for (int i = 0; i < 3; i++)
			ret[i] -= offset[i];

		// convert to requested unit;
		MagnetoUnits.Gauss.convertTo(ret, unit);
	}
	
	/**
	 * fetches measurement in continuous measurement mode
	 * 
	 * @param ret
	 */
	protected void fetchContinuousMeasurementMode(int[] ret) {
		while (!newDataAvailable())
			Delay.msDelay((long) (1000 / sampleRate.rate()));
		fetchRawMagneto(ret);
	}

	/**
	 * Returns the strength of the magnetic field measured by the sensor in
	 * default units. This is the sum of the strength of the earth magnetic field
	 * at the current location and any disturbances around.
	 * 
	 * @return
	 */
	public float fetchFieldStrength() {
		return fetchFieldStrength(magnetoUnit);
	}

	/**
	 * Returns the strength of the magnetic field measured by the sensor in
	 * specified units. This is the sum of the strength of the earth magnetic
	 * field at the current location and any disturbances around.
	 * 
	 * @return
	 */
	public float fetchFieldStrength(MagnetoUnits unit) {
		float[] temp = new float[3];
		float strength = 0;
		fetchAllMagneto(temp, unit);
		for (int i = 0; i < 3; i++)
			strength += Math.pow(temp[i], 2);
		return (float) Math.sqrt(strength);
	}

	/**
	 * Returns the raw values from the data registers of the sensor
	 * 
	 * @param ret
	 */
	protected void fetchRawMagneto(int[] ret) {
		// The order of data registers seems to be X,Z,Y. I do not know if this is
		// typical for my sensor of that this is common to all sensors (Aswin).
		getData(REG_MAGNETO, buf, 6);
		ret[0] = EndianTools.decodeShortBE(buf, 0);
		ret[1] = EndianTools.decodeShortBE(buf, 4);
		ret[2] = EndianTools.decodeShortBE(buf, 2);
	}

	/**
	 * fetches measurement in single measurement mode
	 * 
	 * @param ret
	 */
	protected void fetchSingleMeasurementMode(int[] ret) {
		buf[0] = 0x01;
		sendData(0x02, buf[0]);
		Delay.msDelay(6);
		fetchRawMagneto(ret);
	}

	/**
	 * @return The declination for the current location expressed in radians.
	 */
	public float getDeclination() {
		return declination;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see lejos.robotics.DirectionFinder#getDegreesCartesian()
	 */
	@Override
	public float getDegreesCartesian() {
		return AngleUnits.Radians.convertTo(getHeading(AngleUnits.Radians) - cartesianZero, AngleUnits.DegreesCartesian);
	}

	/**
	 * @return The strength of the magnetic field in milliGauss. One has to set
	 *         this value using setFieldStrength.
	 */
	public float getFieldStrength() {
		return fieldStrength;
	}

	/**
	 * Gives the current heading in the default units for heading
	 * 
	 * @return
	 */
	public float getHeading() {
		return getHeading(angleUnit);
	}

	/**
	 * Gives the current heading in the specified units for heading
	 * 
	 * @return
	 */
	public float getHeading(AngleUnits unit) {
		float[] temp = new float[3];
		fetchAllMagneto(temp, MagnetoUnits.milliGauss);
		return AngleUnits.Radians.convertTo((float) Math.atan2(temp[1], temp[0]) - declination, unit);
	}

	/**
	 * @return The inclination for the current location expressed in radians.
	 */
	public float getInclination() {
		return inclination;
	}

	/**
	 * Returns the default unit that this class uses for reporting measurements
	 * (Gauuss or milliGauss)
	 */
	public MagnetoUnits getMagnetoUnit() {
		return magnetoUnit;
	}

	/**
	 * @return Returns the measurement mode of the sensor (normal, positive bias
	 *         or negative bias).
	 *         <p>
	 *         positive and negative bias mode should only be used for testing the
	 *         sensor.
	 */
	protected MeasurementMode getMeasurementMode() {
		return measurementMode;
	}

	/**
	 * @return The operating mode of the sensor (single measurement, continuous or
	 *         Idle)
	 */
	public OperatingMode getOperatingMode() {
		return operatingMode;
	}

	

	/**
	 * @return The dynamic range of the sensor.
	 */
	public Range getRange() {
		return range;
	}

	/**
	 * @return The refresh rate of the dataregisters of the sensor. Only valid
	 *         when in continuous measurement mode.
	 */
	public SampleRate getSampleRate() {
		return sampleRate;
	}

	/**
	 * @return The number of samples that the sensor takes for one measurement
	 */
	public Samples getSamples() {
		return samples;
	}

	
	@Override
	public String getProductID() {
		return "HMC5883L";
	}

	/**
	 * @return The margin that is used to determine if there are magnetical
	 *         disturbances around.
	 */
	public float getUndesturbedMargin() {
		return margin;
	}
	
	@Override
	public String getVendorID() {
		return "Dexter";
	}

	@Override
	public String getVersion() {
		return "1.0";
	}

	/**
	 * Returns an indication of disturbances of the earth magnetic field. This
	 * method can be used to decide if heading information is trustworthy.
	 * <p>
	 * One must set both the field Strength of the current location (@see
	 * setfieldStrength()) and a margin (@see setmargin()) to use this method.
	 * 
	 * @return TRUE indicates a magnetic disturbance.
	 */
	public boolean isDisturbed() {
		if (fetchFieldStrength(MagnetoUnits.milliGauss) > (1 + margin) * fieldStrength)
			return true;
		if (fetchFieldStrength(MagnetoUnits.milliGauss) < (1 - margin) * fieldStrength)
			return true;
		return false;
	}

	/**
	 * Loads saved offset from memory. If no saved values are found it will load
	 * default values for offset=0;
	 */
	public void loadCalibration() {
		File store = new File(getProductID());
		FileInputStream in = null;
		if (store.exists()) {
			try {
				in = new FileInputStream(store);
				DataInputStream din = new DataInputStream(in);
				for (int i = 0; i < 3; i++) {
					offset[i] = din.readFloat();
				}
				din.close();
			}
			catch (IOException e) {
				System.err.println("Failed to load calibration");
			}
		}
		else {
			for (int i = 0; i < 3; i++) {
				offset[i] = 0;
			}
		}
	}

	/**
	 * Reads the new data ready bit of the status register of the sensor.
	 * 
	 * @return
	 */
	private boolean newDataAvailable() {
		getData(REG_STATUS, buf, 1);
		return ((buf[0] & 0x01) != 0);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see lejos.robotics.DirectionFinder#resetCartesianZero()
	 */
	@Override
	public void resetCartesianZero() {
		cartesianZero = getHeading(AngleUnits.Radians);
	}

	/**
	 * Saves the offset in memory.
	 */
	public void saveCalibration() {
		File store = new File(getProductID());
		FileOutputStream out = null;
		if (store.exists())
			store.delete();
		try {
			out = new FileOutputStream(store);
		}
		catch (FileNotFoundException e) {
			System.err.println("Failed to save calibration");
			System.exit(1);
		}
		DataOutputStream dataOut = new DataOutputStream(out);
		try {
			for (int i = 0; i < 3; i++) {
				dataOut.writeFloat(offset[i]);
			}
			out.close();
		}
		catch (IOException e) {
			System.err.println("Failed to save calibration");
			System.exit(1);
		}
	}

	/**
	 * @param declination
	 *          The declination is used to convert the magnetic north to true
	 *          north.
	 *          <p>
	 *          Set the declination for your location at
	 *          http://magnetic-declination.com/ Declination is expressed in
	 *          degrees and minutes like: 1° 55'
	 */
	public void setDeclination(int degrees, int minutes) {
		this.declination = (float) Math.toRadians(degrees + minutes / 60.0);
	}

	/**
	 * Sets the field strength for your location in milliGauss
	 * 
	 * @param fieldStrength
	 *          Get the field strength for your location at
	 *          http://magnetic-declination.com/ (100 nT =1 milliGuass)
	 *          <p>
	 *          The field strength is used to determine if there are magnetical
	 *          disturbances influencing the measurements.
	 */
	public void setFieldStrength(float fieldStrength) {
		this.fieldStrength = fieldStrength;
	}

	/**
	 * Sets the inclination for your location Get the inclination for your
	 * location at http://magnetic-declination.com/
	 * 
	 * @param inclination
	 *          Inclination is expressed in degrees and minutes like: 1° 55'
	 */
	public void setInclination(int degrees, int minutes) {
		this.inclination = (float) Math.toRadians(degrees + minutes / 60.0);
	}

	/**
	 * Sets the default unit (Gauss, milliGauss) for reporting measurements
	 * 
	 * @param magnetoUnit
	 */
	public void setMagnetoUnit(MagnetoUnits magnetoUnit) {
		this.magnetoUnit = magnetoUnit;
	}

	/**
	 * @param measurementMode
	 *          Sets the measurement mode of the sensor.
	 */
	protected void setMeasurementMode(MeasurementMode measurementMode) {
		this.measurementMode = measurementMode;
		configureSensor();
	}

	/**
	 * Sets the operating mode of the sensor
	 * 
	 * @param operatingMode
	 *          Continuous is normal mode of operation
	 *          <p>
	 *          SingleMeasurement can be used to conserve energy or to increase
	 *          maximum measurement rate
	 *          <p>
	 *          Idle is to stop the sensor and conserve energy
	 */
	public void setOperatingMode(OperatingMode operatingMode) {
		this.operatingMode = operatingMode;
		configureSensor();
	}

	/**
	 * Sets the dynamic range of the sensor (1.3 Gauss is default).
	 * 
	 * @param range
	 */
	public void setRange(Range range) {
		this.range = range;
		configureSensor();
	}

	/**
	 * Sets the refresh rate for the dataregisters of the sensor whe in continuous
	 * measurement mode.
	 * 
	 * @param sampleRate
	 */
	public void setSampleRate(SampleRate sampleRate) {
		this.sampleRate = sampleRate;
		configureSensor();
	}

	/**
	 * Sets the number of samples that the sensor takes internally for one
	 * measurement.
	 * 
	 * @param samples
	 */
	public void setSamples(Samples samples) {
		this.samples = samples;
		configureSensor();
	}

	/**
	 * @param margin
	 *          The margin that is used to determine if there are magnetical
	 *          disturbances around.
	 *          <p>
	 *          0.1 is equivalent to a margin of 10% up or down.
	 */
	public void setUndesturbedMargin(float margin) {
		this.margin = margin;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see lejos.robotics.DirectionFinder#startCalibration().
	 * To calibrate the compass make a few full circles around all three axis. Avoid 
	 * magnetical disutbances while calibrating.
	 */
	@Override
	public void startCalibration() {
		for (int i = 0; i < 3; i++) {
			min[i] = 0;
			max[i] = 0;
		}
		calibrating = true;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see lejos.robotics.DirectionFinder#stopCalibration() Calibration settings
	 * are automaticly stored in memory. Stored calibration settings are loaded
	 * upon startup. Therefore one has to calibrate the compass just once. <p> One
	 * should recalibrate after changing the design of the robot.
	 */
	@Override
	public void stopCalibration() {
		for (int i = 0; i < 3; i++) {
			offset[i] = (float) ((max[i] - min[i]) / 2.0) + min[i];
		}
		saveCalibration();
		calibrating = false;
	}

	/**
	 * Self-test routine of the sensor.
	 * 
	 * @return An array of boolean values. A true indicates the sensor axis is
	 *         working properly.
	 */
	public boolean[] test() {
		boolean[] ret = new boolean[3];

		// store current settings;
		MeasurementMode currentMode = this.getMeasurementMode();
		Range currentRange = this.getRange();
		OperatingMode currentOperatingMode = this.getOperatingMode();

		// modify settings for testing;
		measurementMode = MeasurementMode.PositiveBias;
		range = Range._4_7Ga;
		operatingMode = OperatingMode.SingleMeasurement;
		configureSensor();

		// get measurement
		buf[0] = 0x01;
		sendData(0x02, buf[0]);
		Delay.msDelay(6);
		fetchRawMagneto(raw);

		// test for limits;
		for (int i = 0; i < 3; i++) {
			if (raw[i] > 243 && raw[i] < 575)
				ret[i] = true;
			else
				ret[i] = false;
		}

		// restore settings;
		measurementMode = currentMode;
		range = currentRange;
		operatingMode = currentOperatingMode;
		configureSensor();

		return ret;
	}

	/**
	 * Measurement modes supported by the sensor. Normal mode should be used
	 * unless testing the sensor.
	 */
	protected enum MeasurementMode {
		Normal(0x00), PositiveBias(0x01), NegativeBias(0x02);

		protected int	code;

		MeasurementMode(int code) {
			this.code = code;
		}

		public int code() {
			return code;
		}
	}

	/**
	 * Operating modes supported by the sensor. In continuous mode samples are
	 * taken continuously. This should be the normal mode of operation.
	 * <p>
	 * To conserve power one can use single measurement mode. In this mode a
	 * measurement is only made at request. After the measurement the sensor goes
	 * into an idle state.
	 */
	public enum OperatingMode {
		Continuous(0x00), SingleMeasurement(0x01), Idle(0x02);

		private final int	code;

		OperatingMode(int code) {
			this.code = code;
		}

		public int code() {
			return code;
		}
	}

	/**
	 * Dynamic ranges supported by the sensor.
	 */
	public enum Range {
		_0_88Ga(0x00, 1370), _1_3Ga(0x01, 1090), _1_9Ga(0x02, 820), _2_5Ga(0x03, 660), _4_0Ga(0x04, 440), _4_7Ga(0x05, 390), _5_6Ga(0x06, 330), _8_1Ga(0x07, 230);

		private final int	code;
		private final int	gain;

		Range(int code, int gain) {
			this.code = code << 5;
			this.gain = gain;
		}

		public int code() {
			return code;
		}

		public int gain() {
			return gain;
		}
	}

	/**
	 * External sample rates supported by the sensor
	 */
	public enum SampleRate {
		_0_75Hz(0x0, 0.75), _1_5Hz(0x01, 1.5), _3_0Hz(0x02, 3.0), _7_5Hz(0x03, 7.5), _15Hz(0x04, 15.0), _30Hz(0x05, 30.0), _75Hz(0x06, 75.0);

		private final int			code;
		private final double	rate;

		SampleRate(int code, double rate) {
			this.code = code << 2;
			this.rate = rate;
		}

		public int code() {
			return code;
		}

		public double rate() {
			return rate;
		}
	}

	/**
	 * The number of internal samples the sensor uses for one measurement
	 */
	public enum Samples {
		_1(0x00), _2(0x01), _4(0x02), _8(0x03);

		private final int	code;

		Samples(int code) {
			this.code = code << 5;
		}

		public int code() {
			return code;
		}
		
		public enum MagnetoUnits {
			milliGauss,
			Gauss;
			public float convertTo(float value, MagnetoUnits unit) {
				if (this==unit) return value;
				if (this==MagnetoUnits.milliGauss && unit==MagnetoUnits.Gauss) return value/1000.0f; 
				if (this==MagnetoUnits.Gauss && unit==MagnetoUnits.milliGauss) return value*1000.0f; 
				return Float.NaN;
			}
			
			public void convertTo(float[] values, MagnetoUnits unit) {
				for (int i=0;i<3;i++)
					values[i]=convertTo(values[i],unit);
			}
			
		}
		
		public enum AngleUnits {
			Degrees,
			Radians,
			DegreesCartesian;
			public float convertTo(float value, AngleUnits unit) {
				if (this==AngleUnits.Degrees && unit==AngleUnits.Radians) return (float) Math.toRadians(value);
				if (this==AngleUnits.Degrees && unit==AngleUnits.DegreesCartesian) return -value;
				if (this==AngleUnits.Radians && unit==AngleUnits.Degrees) return (float) Math.toDegrees(value);
				if (this==AngleUnits.Radians && unit==AngleUnits.DegreesCartesian) return -(float)Math.toDegrees(value);
				if (this==AngleUnits.DegreesCartesian && unit==AngleUnits.Degrees) return -value;
				if (this==AngleUnits.DegreesCartesian && unit==AngleUnits.Radians) return (float) Math.toRadians(-value);
				return value;
			}
			
		}
	}

}
