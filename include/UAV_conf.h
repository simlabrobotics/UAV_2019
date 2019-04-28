#ifndef __UAV_CONF_H__
#define __UAV_CONF_H__

#include <rlab/type.h>
#include <rxSDK\/rxUserProperty.h>
#include <rlab/utils/rParseUtil.h>

#define UAVCONF_READ_PROP_BOOLEAN(prop_name, var_name)  \
	prop = getProperty(prop_name);						\
		if (prop) {										\
			if (_tcsicmp(prop, _T("true")) == 0 ||		\
				_tcsicmp(prop, _T("yes")) == 0)			\
				(##var_name) = true;					\
			else										\
				(##var_name) = false;					\
		}

#define UAVCONF_READ_PROP_INT(prop_name, var_name)		\
	prop = getProperty(prop_name);						\
	if (prop) (##var_name) = _tstoi(prop)

#define UAVCONF_READ_PROP_REAL(prop_name, var_name)		\
	prop = getProperty(prop_name);						\
	if (prop) (##var_name) = _tstof(prop)

class UAVConf : public rxUserProperty
{
public:
	UAVConf() 
		: _bLoaded(false)
		, _delT(0.005)
		, _use_AMBSDynamics(true)
		, _enable_aml(true)
		, _aml_filepath(_T(""))
		, _aml_name(_T("MyRobot"))
		, _aml_isStatic(false)
		, _enable_eml(false)
		, _eml_filepath(_T(""))
		, _eml_name(_T("Environment"))
		, _enable_control(false)
		, _control_filepath(_T(""))
		, _control_name(_T("MyControl"))
		, _plot_position_enabled(false)
		, _plot_velocity_enabled(false)
		, _plot_acc_sensor_enabled(false)
		, _plot_gyro_sensor_enabled(false)
		, _plot_gps_sensor_enabled(false)
		, _plot_slippage_enabled(false)
		, _plot_slippage_tracked_enabled(false)
		, _plot_lateral_force_enabled(false)
		, _net_port_simulation(5150)
		, _net_port_control(5250)
		, _debug_control_local_enable(false)
		, _hmap_path(_T(""))
		, _hmap_map_width(0.0f)
		, _hmap_map_length(0.0f)
		, _hmap_map_height_min(0.0f)
		, _hmap_map_height_max(0.0f)
	{
		_eml_r0.setZero();
		_eml_R0.setIdentity();
		_aml_R0.setIdentity();
		_aml_r0.setZero();

		_wp_line_width = 4;
		_wp_offset_z = 1.0;
		_wp_line_color[0] = _wp_line_color[1] = _wp_line_color[2] = _wp_line_color[3] = 1.0f;
	}
		
	~UAVConf() {}

	void parseConfig(const string_type& filepath)
	{
		loadProperty(filepath);

		const TCHAR* prop;

		prop = getProperty(_T("time_step"));
		if (prop)
			_delT = _tstof(prop);

		UAVCONF_READ_PROP_BOOLEAN(_T("use_AMBSDynamics"), _use_AMBSDynamics);

		UAVCONF_READ_PROP_BOOLEAN(_T("aml_enable"), _enable_aml);

		UAVCONF_READ_PROP_BOOLEAN(_T("aml_isStatic"), _aml_isStatic);

		prop = getProperty(_T("aml_name"));
		if (prop)
			_aml_name = prop;

		prop = getProperty(_T("aml_path"));
		if (prop)
			_aml_filepath = prop;

		prop = getProperty(_T("aml_r0"));
		if (prop) {
			float r[3];
			parse_vector(r, 3, prop);
			for (int i=0; i<3; i++)
				_aml_r0[i] = r[i];
		}

		prop = getProperty(_T("aml_R0"));
		if (prop) {
			float R[9];
			parse_vector(R, 9, prop);
			_aml_R0(0, 0) = R[0]; _aml_R0(0, 1) = R[1]; _aml_R0(0, 2) = R[2];
			_aml_R0(1, 0) = R[3]; _aml_R0(1, 1) = R[4]; _aml_R0(1, 2) = R[5];
			_aml_R0(2, 0) = R[6]; _aml_R0(2, 1) = R[7]; _aml_R0(2, 2) = R[8];
		}

		UAVCONF_READ_PROP_BOOLEAN(_T("eml_enable"), _enable_eml);

		prop = getProperty(_T("eml_name"));
		if (prop)
			_eml_name = prop;

		prop = getProperty(_T("eml_path"));
		if (prop)
			_eml_filepath = prop;

		prop = getProperty(_T("eml_r0"));
		if (prop) {
			float r[3];
			parse_vector(r, 3, prop);
			for (int i=0; i<3; i++)
				_eml_r0[i] = r[i];
		}

		prop = getProperty(_T("eml_R0"));
		if (prop) {
			float R[9];
			parse_vector(R, 9, prop);
			_eml_R0(0, 0) = R[0]; _eml_R0(0, 1) = R[1]; _eml_R0(0, 2) = R[2];
			_eml_R0(1, 0) = R[3]; _eml_R0(1, 1) = R[4]; _eml_R0(1, 2) = R[5];
			_eml_R0(2, 0) = R[6]; _eml_R0(2, 1) = R[7]; _eml_R0(2, 2) = R[8];
		}

		UAVCONF_READ_PROP_BOOLEAN(_T("control_enable"), _enable_control);

		prop = getProperty(_T("control_name"));
		if (prop)
			_control_name = prop;

		prop = getProperty(_T("control_path"));
		if (prop)
			_control_filepath = prop;

		UAVCONF_READ_PROP_INT(_T("net_port_simulation"), _net_port_simulation);
		UAVCONF_READ_PROP_INT(_T("net_port_control"), _net_port_control);

		UAVCONF_READ_PROP_BOOLEAN(_T("plot_position"), _plot_position_enabled);
		UAVCONF_READ_PROP_BOOLEAN(_T("plot_velocity"), _plot_velocity_enabled);
		UAVCONF_READ_PROP_BOOLEAN(_T("plot_gyro_sensor"), _plot_gyro_sensor_enabled);
		UAVCONF_READ_PROP_BOOLEAN(_T("plot_acc_sensor"), _plot_acc_sensor_enabled);
		UAVCONF_READ_PROP_BOOLEAN(_T("plot_gps_sensor"), _plot_gps_sensor_enabled);
		UAVCONF_READ_PROP_BOOLEAN(_T("plot_slippage"), _plot_slippage_enabled);
		UAVCONF_READ_PROP_BOOLEAN(_T("plot_slippage_tracked"), _plot_slippage_tracked_enabled);
		UAVCONF_READ_PROP_BOOLEAN(_T("plot_lateral_force"), _plot_lateral_force_enabled);

		UAVCONF_READ_PROP_BOOLEAN(_T("debug_control_local_enable"), _debug_control_local_enable);

		prop = getProperty(_T("HMAP_PATH"));
		if (prop)
			_hmap_path = prop;

		UAVCONF_READ_PROP_REAL(_T("HMAP_W"), _hmap_map_width);
		UAVCONF_READ_PROP_REAL(_T("HMAP_L"), _hmap_map_length);
		UAVCONF_READ_PROP_REAL(_T("HMAP_H_max"), _hmap_map_height_max);
		UAVCONF_READ_PROP_REAL(_T("HMAP_H_min"), _hmap_map_height_min);

		UAVCONF_READ_PROP_REAL(_T("WP_LINE_WIDTH"), _wp_line_width);
		UAVCONF_READ_PROP_REAL(_T("WP_OFFSET_FROM_GROUND"), _wp_offset_z);
		prop = getProperty(_T("WP_LINE_COLOR"));
		if (prop) {
			float wp_color[4];
			parse_vector(wp_color, 4, prop);
			for (int i = 0; i<4; i++)
				_wp_line_color[i] = wp_color[i];
		}

		_bLoaded = true;
	}
	
	bool isLoaded() { return _bLoaded; }

public:
	rTime delT() const { return _delT; }
	bool useAMBSDynamics() const { return _use_AMBSDynamics; }
	
	bool enableAml() const { return _enable_aml; }
	bool amlIsStatic() const { return _aml_isStatic; }
	const string_type& amlPath() const { return _aml_filepath; }
	const string_type& amlName() const { return _aml_name; }
	const Displacement& aml_r0() const { return _aml_r0; }
	const Rotation& aml_R0() const { return _aml_R0; }
	const dVector& aml_q0() const { return _aml_q0; }
	
	bool enableEml() const { return _enable_eml; }
	const string_type& emlPath() const { return _eml_filepath; }
	const string_type& emlName() const { return _eml_name; }
	const Displacement& eml_r0() const { return _eml_r0; }
	const Rotation& eml_R0() const { return _eml_R0; }

	bool enableControl() const { return _enable_control; }
	const string_type& controlPath() const { return _control_filepath; }
	const string_type& controlName() const { return _control_name; }

	bool enablePlotPosition() const { return _plot_position_enabled; }
	bool enablePlotVelocity() const { return _plot_velocity_enabled; }
	bool enablePlotAccSensor() const { return _plot_acc_sensor_enabled; }
	bool enablePlotGyroSensor() const { return _plot_gyro_sensor_enabled; }
	bool enablePlotGPSSensor() const { return _plot_gps_sensor_enabled; }
	bool enablePlotSlippage() const { return _plot_slippage_enabled; }
	bool enablePlotSlippageTracked() const { return _plot_slippage_tracked_enabled; }
	bool enablePlotLateralForce() const { return _plot_lateral_force_enabled; }
	
	unsigned short netSimulationPort() const { return _net_port_simulation; }
	unsigned short netControlPort() const { return _net_port_control; }

	bool enableDebugControlLocal() const { return _debug_control_local_enable; }

	const string_type& heightmapPath() const { return _hmap_path; }
	const float heightmapMapWidth() const { return _hmap_map_width; }
	const float heightmapMapLength() const { return _hmap_map_length; }
	const float heightmapMapHeightMax() const { return _hmap_map_height_max; }
	const float heightmapMapHeightMin() const { return _hmap_map_height_min; }

	const float wpLineWidth() const { return _wp_line_width; }
	const float wpOffsetFromGround() const { return _wp_offset_z; }
	const float* wpLineColor() const { return _wp_line_color; }

private:
	bool _bLoaded;

	// application
	rTime _delT;
	bool _use_AMBSDynamics;
	// robot
	bool _enable_aml;
	bool _aml_isStatic;
	string_type _aml_filepath;
	string_type _aml_name;
	Displacement _aml_r0;
	Rotation _aml_R0;
	dVector _aml_q0;
	// environment
	bool _enable_eml;
	string_type _eml_filepath;
	string_type _eml_name;
	Displacement _eml_r0;
	Rotation _eml_R0;
	// control
	bool _enable_control;
	string_type _control_filepath;
	string_type _control_name;
	// plot
	bool _plot_position_enabled;
	bool _plot_velocity_enabled;
	bool _plot_acc_sensor_enabled;
	bool _plot_gyro_sensor_enabled;
	bool _plot_gps_sensor_enabled;
	bool _plot_slippage_enabled;
	bool _plot_slippage_tracked_enabled;
	bool _plot_lateral_force_enabled;
	// network
	unsigned short _net_port_simulation;
	unsigned short _net_port_control;
	// debug
	bool _debug_control_local_enable;
	// height map
	string_type _hmap_path;
	float _hmap_map_width;
	float _hmap_map_length;
	float _hmap_map_height_min;
	float _hmap_map_height_max;
	// waypoints
	float _wp_line_width;
	float _wp_offset_z;
	float _wp_line_color[4];
};

#endif // __UAV_CONF_H__
