#include "Myekf2.h"
#include <fstream>
#include <time.h>
#include <stdio.h>

class Ekf2;
std::ifstream read1("../data1/imu_data.txt");
std::ifstream read2("data1/gps_data.txt");
std::ifstream read3("../data1/mag_data.txt");
std::ifstream read4("../data1/baro_data.txt");
// std::ifstream read1("../data1/imu.txt");
// std::ifstream read2("data1/gps.txt");
// std::ifstream read3("../data1/mag.txt");
// std::ifstream read4("../data1/baro.txt");
std::ofstream euler_estimator("../results/euler_estimator.txt");
std::ofstream position_estimator("../results/position_estimator.txt");
std::ofstream velocity_estimator("../results/velocity_estimator.txt");

bool bReadGPS, bReadMag, bReadBaro;


namespace ekf2
{
Ekf2 *instance = nullptr;
}

Ekf2::Ekf2():
	_ekf(),
	_params(_ekf.getParamHandle())
{

}

Ekf2::~Ekf2()
{

}


void Ekf2::print_status()
{
	printf("local position OK %s", (_ekf.local_position_is_valid()) ? "[YES]" : "[NO]");
	printf("global position OK %s", (_ekf.global_position_is_valid()) ? "[YES]" : "[NO]");
}

void Ekf2::task_main()
{
	// initialise parameter cache// TODO
	//updateParams();
//	std::ifstream read1("data/imu_data.txt");

	float gyro_integral_dt = 0;
	float accelerometer_integral_dt = 0;
	float last_IMUtime = 0;
    float now = 0;
    double gps_time_us, lat, lon, alt, fix_type, eph, epv, s_variance_m_s, vel_m_s, vel_n_m_s, vel_e_m_s, vel_d_m_s, vel_ned_valid, satellites_used;
    double mag_time_us_read, magx, magy, magz;
    double baro_time_us_read, baroHeight, baroHeight_origin = 0.0f;

	//while (!_task_should_exit && !read1.eof() && !read2.eof() && !read3.eof() && !read4.eof()) {
	while (!_task_should_exit && !read1.eof() &&  !read3.eof() && !read4.eof()) {

		bool isa = true;
		bool mag_updated = false;
		bool baro_updated = false;
		bool gps_updated = false;
		bool vehicle_status_updated = false;

		// long gyro_integral_dt = 0.01;
		// // in replay mode we are getting the actual timestamp from the sensor topic
			
		read1 >> now;	//us

		// // push imu data into estimator
		float gyro_integral[3],gyro_rad[3];
		read1 >> gyro_rad[0];	read1 >> gyro_rad[1];	read1 >> gyro_rad[2];
		//printf("gyro:%lf,%lf,%lf,%lf s\n", gyro_rad[0], gyro_rad[1], gyro_rad[2],gyro_integral_dt);

		read1 >> gyro_integral_dt;
		gyro_integral_dt /= 1.e6f;	//s

		float temp;
		read1 >> temp;	

		gyro_integral[0] = gyro_rad[0] * gyro_integral_dt;
		gyro_integral[1] = gyro_rad[1] * gyro_integral_dt;
		gyro_integral[2] = gyro_rad[2] * gyro_integral_dt;

		float accel_integral[3],accelerometer_m_s2[3];
		read1 >> accelerometer_m_s2[0];	read1 >> accelerometer_m_s2[1];	read1 >> accelerometer_m_s2[2];
		//printf("accelerometer_m_s2:%lf,%lf,%lf\n", accelerometer_m_s2[0], accelerometer_m_s2[1], accelerometer_m_s2[2]);

		read1 >> accelerometer_integral_dt;
		accelerometer_integral_dt /=1.e6f;

		accel_integral[0] = accelerometer_m_s2[0] * accelerometer_integral_dt;
		accel_integral[1] = accelerometer_m_s2[1] * accelerometer_integral_dt;
		accel_integral[2] = accelerometer_m_s2[2] * accelerometer_integral_dt;
		_ekf.setIMUData(now, gyro_integral_dt * 1.e6f, accelerometer_integral_dt * 1.e6f,
				gyro_integral, accel_integral);		
		last_IMUtime = now;

		if(bReadMag)
		{
			read3 >> mag_time_us_read;	//us
			// float temp; read3>>temp;read3>>temp;
			read3 >> magx;
			read3 >> magy;
			read3 >> magz;
			//magx /= 100.0f;magy /= 100.0f;magz /= 100.0f;
			// read3>>temp;
			bReadMag = false;		
		}
		if(mag_time_us_read  < now)
		{
			mag_updated = true;
			bReadMag = true;
		}
		if(mag_updated)
		{
			_timestamp_mag_us = mag_time_us_read ;

			// If the time last used by the EKF is less than specified, then accumulate the
			// data and push the average when the 50msec is reached.
			_mag_time_sum_ms += _timestamp_mag_us / 1000.0f;
			_mag_sample_count++;
			_mag_data_sum[0] += magx;
			_mag_data_sum[1] += magy;
			_mag_data_sum[2] += magz;
			uint32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;
			
			if (mag_time_ms - _mag_time_ms_last_used > _params->sensor_interval_min_ms) {
				float mag_sample_count_inv = 1.0f / (float)_mag_sample_count;
				float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv, _mag_data_sum[1] *mag_sample_count_inv, _mag_data_sum[2] *mag_sample_count_inv};
				_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);
				//printf("mag: %f %f %d %f %f %f\n",now,mag_time_us_read,mag_time_ms,
				//		_mag_data_sum[0],_mag_data_sum[1],_mag_data_sum[2]);
				_mag_time_ms_last_used = mag_time_ms;
				_mag_time_sum_ms = 0;
				_mag_sample_count = 0;
				_mag_data_sum[0] = 0.0f;
				_mag_data_sum[1] = 0.0f;
				_mag_data_sum[2] = 0.0f;	
			}		
		}


		if(bReadBaro)
		{
			read4 >> baro_time_us_read;	//us
			read4 >> baroHeight ;
			float temp; 
			read4>>temp;read4>>temp;read4>>temp;
			// baroHeight /= 2.0f;
			// if(baroHeight_origin == 0)
			// 	baroHeight_origin = baroHeight;
			// baroHeight -= baroHeight_origin;
			bReadBaro= false;		
		}
		if(baro_time_us_read  <now)
		{
			baro_updated = true;
			bReadBaro = true;
		}
		if(baro_updated)
		{
				_timestamp_balt_us = baro_time_us_read;

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the 50msec is reached.
				_balt_time_sum_ms += _timestamp_balt_us / 1000;
				_balt_sample_count++;
				_balt_data_sum += baroHeight;
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

				if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)_params->sensor_interval_min_ms) {
					float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;
				//printf("baro: %f %f %d %f\n",now,baro_time_us_read,balt_time_ms,
				//		balt_data_avg);					
					_ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);
					_balt_time_ms_last_used = balt_time_ms;
					_balt_time_sum_ms = 0;
					_balt_sample_count = 0;
					_balt_data_sum = 0.0f;

				}			
		}

		if(bReadGPS)
		{
			read2 >> gps_time_us;	//us
			float temp1; read2>>temp1;
			read2 >> lat;
			read2 >> lon;
			read2 >> alt;
			read2>>temp1;
			read2>>s_variance_m_s;read2>>temp1;read2>>eph;read2>>epv;
			read2>>temp1;read2>>temp1;read2>>temp1;read2>>temp1;
			read2 >> vel_m_s;
			read2>>vel_n_m_s;read2>>vel_e_m_s;read2>>vel_d_m_s;
			read2>>temp1;read2>>temp1;
			read2>>fix_type;read2>>vel_ned_valid;
			read2>>satellites_used;
			bReadGPS = false;		
		}
		if(gps_time_us   < now)
		{
			gps_updated = true;
			bReadGPS = true;
		}
		if(gps_updated)
		{
			struct gps_message gps_msg = {};
			gps_msg.time_usec = gps_time_us;
			gps_msg.lat = lat;
			gps_msg.lon = lon;
			gps_msg.alt = alt;
			//printf("time now: %lf\n", now);
			//printf("gps: %ld, %d, %d, %d\n",gps_msg.time_usec,gps_msg.lat,gps_msg.lon,gps_msg.alt);
			gps_msg.fix_type = fix_type;
			gps_msg.eph = eph;
			gps_msg.epv = epv;
			gps_msg.sacc = s_variance_m_s;
			gps_msg.vel_m_s = vel_m_s;
			gps_msg.vel_ned[0] = vel_n_m_s;	//no gps ned vel
			gps_msg.vel_ned[1] = vel_e_m_s;
			gps_msg.vel_ned[2] = vel_d_m_s;
			gps_msg.vel_ned_valid = vel_ned_valid;
			gps_msg.nsats = satellites_used;
			//TODO add gdop to gps topic
			gps_msg.gdop = 0.0f;

			_ekf.setGpsData(gps_msg.time_usec, &gps_msg);
		}

		//usleep(500000);



		//run the EKF update and output
		if (_ekf.update()) {
		

			matrix::Quaternion<float> q;
			_ekf.copy_quaternion(q.data());

			float velocity[3];
			_ekf.get_velocity(velocity);
			//printf("velocity: %lf,%lf,%lf\n", velocity[0], velocity[1], velocity[2]);
			velocity_estimator<< now <<" "<<velocity[0] <<" "<<velocity[1] <<" "
            <<-velocity[2] <<" "<<std::endl;

			float gyro_rad[3];

			{
				// generate control state data
				float gyro_bias[3] = {};
				_ekf.get_gyro_bias(gyro_bias);
				gyro_rad[0] = gyro_rad[0] - gyro_bias[0];
				gyro_rad[1] = gyro_rad[1] - gyro_bias[1];
				gyro_rad[2] = gyro_rad[2] - gyro_bias[2];

				// Velocity in body frame
				Vector3f v_n(velocity);
				matrix::Dcm<float> R_to_body(q.inversed());
				Vector3f v_b = R_to_body * v_n;


				// Local Position NED
				float position[3];
				_ekf.get_position(position);
				//printf("position: %lf,%lf,%lf\n", position[0], position[1], position[2]);
				position_estimator<< now <<" "<<position[0] <<" "<<position[1] <<" "
				<<-position[2] <<" "<<std::endl;
				// Attitude quaternion
				//q.copyTo(ctrl_state.q);

				//_ekf.get_quat_reset(&ctrl_state.delta_q_reset[0], &ctrl_state.quat_reset_counter);

				// Acceleration data
				matrix::Vector<float, 3> acceleration(accelerometer_m_s2);

				float accel_bias[3];
				_ekf.get_accel_bias(accel_bias);
				// ctrl_state.x_acc = acceleration(0) - accel_bias[0];
				// ctrl_state.y_acc = acceleration(1) - accel_bias[1];
				// ctrl_state.z_acc = acceleration(2) - accel_bias[2];

				// // compute lowpass filtered horizontal acceleration
				acceleration = R_to_body.transpose() * acceleration;
				// _acc_hor_filt = 0.95f * _acc_hor_filt + 0.05f * sqrtf(acceleration(0) * acceleration(0) +
				// 		acceleration(1) * acceleration(1));
				// ctrl_state.horz_acc_mag = _acc_hor_filt;

				// ctrl_state.airspeed_valid = false;

			}
			
			// generate vehicle local position data

			float pos[3] = {};
			// Position of body origin in local NED frame
			_ekf.get_position(pos);
			//printf("%f  %f  %f\n", pos[0],pos[1],pos[2]);

			// Velocity of body origin in local NED frame (m/s)

			// TODO: better status reporting
	

			// Position of local NED origin in GPS / WGS84 frame
			
			// true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
			//_ekf.get_ekf_origin(&lpos.ref_timestamp, &ekf_origin, &lpos.ref_alt);
		
			// The rotation of the tangent plane vs. geographical north
			matrix::Eulerf euler(q);
			//printf("euler: %f  %f  %f\n", euler.phi(),euler.theta(),euler.psi());
				euler_estimator<< now <<" "<<euler.phi() <<" "<<euler.theta() <<" "
				<<euler.psi() <<" "<<std::endl;			
			// TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
			Vector3f pos_var, vel_var;
			_ekf.get_pos_var(pos_var);
			_ekf.get_vel_var(vel_var);
			//printf("pos_var: %lf,%lf,%lf\n", pos_var(0), pos_var(1), pos_var(2) );
			//printf("vel_var: %lf,%lf,%lf\n", vel_var(0), vel_var(1), vel_var(2) );

		} 

	}
	printf("end\n");


}

int main(int argc, char *argv[])
{
	printf("begin\n");
	bReadGPS = true;
	Ekf2* _ekf2 = new Ekf2();
	//_ekf2->print_status();
	_ekf2->task_main();


	return 1;
}
