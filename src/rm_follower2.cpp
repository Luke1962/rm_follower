/*
Nodo che sottoscrive   dnn_detect::DetectedObjectArray
e genera cmd_vel, raspicam_tilt


Build : catkin_make  -DCATKIN_WHITELIST_PACKAGES="rm_follower"
        richiede ros_arduino per i messaggi
Launch:  roslaunch rm_follower rm_follower.launch
*/

#define MINIMUMSPEED 0.15 //velocità minima per spostarsi linearmente

//#include <assert.h>
#include <sys/time.h>
//#include <unistd.h>

#include <ros/ros.h>

// Actionlib
// =======================================================================
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

// This includes action message generated from the Move.action file.
// This is a header generated automatically from the AveragingAction.msg file.
// For more information on message definitions, see the msg page.
// il nome dell'include è [Nome senza .action]Action.h
#include <rm_follower/MoveAction.h>

//================================================================================

#include <boost/algorithm/string.hpp>
#include <list>
#include <string>

#include <condition_variable>
#include <mutex>
#include <thread>

#include "dnn_detect/DetectedObject.h"
#include "dnn_detect/DetectedObjectArray.h"
#include <geometry_msgs/Point.h>      // posizione persona
#include <geometry_msgs/Twist.h>      // comandi
#include <ros/transport_hints.h>      // per tcp_nodelay
#include <ros_arduino_msgs/Digital.h> //pir
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <visualization_msgs/Marker.h>

//#include "PID_v1.h"
//#include <afsm/fsm.hpp>
#define PI           3.14159265358979323846  /* pi */
using namespace std;

std::condition_variable cond;
std::mutex mutx;

enum status_e {	// STATI
	STATUS_WAIT_FOR_START= 0,
	STATUS_IDLE ,
	STATUS_SEARCHING,
	STATUS_TRACKING
};

/*
 Black: \u001b[30m
Red: \u001b[31m
Green: \u001b[32m
Yellow: \u001b[33m
Blue: \u001b[34m
Magenta: \u001b[35m
Cyan: \u001b[36m
White: \u001b[37m
Reset: \u001b[0m
 
  RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'
*/
class Color{
public:
 
   const char*    RED 		= "\033[31m";
   const char*    GREEN 	= "\033[32m";
   const char*    YELLOW 	= "\033[33m";
   const char*    MAGENTA 	= "\033[35m";
   const char*    CYAN    	= "\033[36m";
  
   const char*    BLUE 		= "\033[34m";
   const char*    BOLD 		= "\033[1m";
   
   const char *     END 		= "\033[0m";
 
};
Color col;

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }
////////////////////////////////////////////////////////////////////////////////////
// mappa 'value' espresso nel range [istart,istop] nel dominio [ostart,ostop]
float remap(float value, float istart, float istop, float ostart, float ostop) {
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
// calcola l'angolo sulla base della distanza in pixel dal centro immagine (> 0 -> Lato destro -> rotazione CW)
float pixel2deg_H(int pixelFromCenter, float cameraHFOV_rad, int imageW ) {
  // https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation
  //float pixelFov_deg = (cameraHFOV_rad*180/2PI)/imageW; // fov per pixel in gradi
  float alfa_rad =atan(  	- 2.0*tan(cameraHFOV_rad / 2)  * pixelFromCenter / imageW   	);
  
  return alfa_rad*180/PI;
  
}
///////////////////////////////////////////////////////////////////////////////////////mnt

#define STOP                                                                   \
  msg_cmdvel.linear.x = 0;                                                     \
  msg_cmdvel.linear.y = 0;                                                     \
  msg_cmdvel.angular.z = 0;                                                    \
  pub_cmdvel.publish(msg_cmdvel);                                              \
  printf("\n\tSTOP\n");

//#define CMDVEL(lin,ang)  ( msg_cmdvel.linear.x = lin;msg_cmdvel.linear.y = 0;msg_cmdvel.angular.z = ang;pub_cmdvel.publish(msg_cmdvel);)   
  //printf("\n\t cmd_vel[%f,%f]", (float)lin, (float)ang);


class Follower {
private:
	//----------------------------------------------------------------------
	// mie aggiunte
	//----------------------------------------------------------------------
	typedef struct {
		float_t posX; //impostato dal cbk delle detection, usato da track
		float_t distanceFromCenter;
		float_t width; 
		ros::Time ts;		
	} detect_t;


	//geometry_msgs::Point detectedPerson; // persona piu vicina
	detect_t lastDetection;
	std::string lastSpeech;
	void speech(string msg);
	void speech_once(string msg);
	void speech_coded_norepeat(string msg);
	float w2dist(int w, int imgW);
	
	float width2distance;
	bool isNewDetection;
	float trackRotationSpeed;
	float searchRotationSpeed;
	float trackLinerarSpeed;

	int thresholdCenter;
	std::string cmdvel_topic;
	std::string camerainfo_topic;

	float min_confidence;				// minimo livello di confidenza
	float tooFarFactor;					// se detection.width < tooFarFactor*InageW  mi devo avvicinare  
	float tooNearFactor;				// se detection.width < tooFarFactor*InageW  mi devo avvicinare 
	int searchRotationDir = 1; 			// direzione di rotazione di ricerca
	float current_search_rotation_speed_rad;
	int lastdetection2stop_sec = 10; 	// secondi dopo l'ultima detection per fermarsi
	int lastDetection2search_sec;

	// Dati webcam
	int imageW;
	int imageH;
	float cameraHFOV; // angolo di visuale orizzontale

	// messaggi ROS
	std_msgs::String msg_chatter;				// parlato
	std_msgs::String msg_speech_coded;
	geometry_msgs::Point msg_personPosition;
	geometry_msgs::Twist msg_cmdvel;
	//ros::Time lastDetection.ts; // quando è occorsa l'ultima detection

	bool debug_onlyRotation= false;				// se true non avanza quando la detection è al centro
	uint8_t pir; 								// true se rileva presenza umana
	#define MIN_CONSECUTIVE_DETECTIONS 3
	int consecutiveDetectionCount =0;
	bool isPersonDetected = false;

	status_e status;


	ros::Time time_previous_status ; //serve a gestire delay minimi tra due stati
	void enter__STATUS_MANAGER(); //gestore stati
	void enter_IDLE_FROM_START();
	void enter_IDLE_FROM_SEARCHING();
	void enter_SEARCHING_FROM_IDLE();
	void enter_SEARCHING_FROM_TRACKING();
	void enter_TRACKING_FROM_SEARCHING();
	void enter_TRACKING_FROM_IDLE();
		
	void speech_coded(const std::string text);

	void CMDVEL(float lin, float ang);
	void moveToTargetPose(float fw, float degL);
	void track(detect_t target); // modulo di inseguimento in base alla posizione 


	void cbk_doppler(const std_msgs::Bool &msg);
	void cbk_camerainfo(const sensor_msgs::CameraInfo);
	void cbk_start_followme(const std_msgs::Bool &msg);
	void cbk_detectedObjects(const dnn_detect::DetectedObjectArray &detectionArray);
	void cbk_detectedPersons(const dnn_detect::DetectedObjectArray &detectionArray);
	
	void publish_Marker(float x, float y);
	
	
	// subscribers -------------------------------
	ros::Subscriber sub_start_followme;
	ros::Subscriber sub_doppler;
	ros::Subscriber sub_camerainfo;
	ros::Subscriber sub_detectedPersons;
	
	// publishers --------------------------------
	
	ros::Publisher pub_chatter;
	ros::Publisher pub_speech_once;
	ros::Publisher pub_speech_coded;
	
	ros::Publisher pub_raspicamtilt;
	ros::Publisher pub_faretto;
	
	ros::Publisher pub_cmdvel;
	ros::Publisher pub_target_pose;

 	ros::Publisher pub_person_marker;
 //----------------------------------------------------------------------
public:

	bool start_followme; 
	Follower(ros::NodeHandle &nh);
	
	float node_current_frequency;
	float node_rate_hz;
};


void Follower::CMDVEL(float lin, float ang){
	msg_cmdvel.linear.x = lin;
	msg_cmdvel.linear.y = 0;
	msg_cmdvel.angular.z = ang;
	pub_cmdvel.publish(msg_cmdvel);
}
void Follower::moveToTargetPose(float fw, float degL){
	msg_cmdvel.linear.x = fw;
	msg_cmdvel.linear.y = 0;
	msg_cmdvel.angular.z = degL;
	pub_target_pose.publish(msg_cmdvel);
}


void Follower::speech(std::string msg) {
  msg_chatter.data = msg;
  pub_chatter.publish(msg_chatter);
  //ROS_INFO("[SPEECH] %s", msg.c_str());
}
void Follower::speech_once(std::string msg) {
  msg_chatter.data = msg;
  pub_speech_once.publish(msg_chatter);
}
void Follower::cbk_camerainfo(sensor_msgs::CameraInfo camerainfo) {
  imageW = camerainfo.width;
  imageH = camerainfo.height;
  ROS_INFO_ONCE("Received Camera_info. Size: %d x %d", imageW, imageH);
  ROS_INFO_ONCE("                      Left region x: [0..%d] Right region: [%d..%d]", imageW/2-thresholdCenter, imageW/2+thresholdCenter , imageW);
    
}

 

 
/////////////////////////////////////////
// Gestione transizioni di stato
/////////////////////////////////////////
	void Follower::enter_IDLE_FROM_START(){
		ROS_INFO_THROTTLE(5.0, "%s[STATUS_IDLE] Waiting for PIR signal%s",col.MAGENTA,col.END);

		speech_coded_norepeat("speech_f_start");
		node_current_frequency = 0.9*node_rate_hz;
		
		status = STATUS_IDLE;	
	}
	
	void Follower::enter_IDLE_FROM_SEARCHING(){
		ROS_INFO("%s[STATUS_SEARCHING ->  STATUS_IDLE ]%s",col.MAGENTA,col.END);	

		STOP;
		speech_coded_norepeat("speech_f_nessuno");		

		status = STATUS_IDLE;
	}
	
	void Follower::enter_SEARCHING_FROM_IDLE(){
		ROS_INFO("%s[STATUS_IDLE -> STATUS_SEARCHING]%s",col.MAGENTA,col.END);
		
		speech_coded_norepeat("speech_f_movimento");
		lastDetection.ts =	ros::Time::now(); // per non farlo tornare subito in STATUS_IDLE
		node_current_frequency = 0.7*node_rate_hz;
		current_search_rotation_speed_rad = (float)searchRotationSpeed * searchRotationDir;
		CMDVEL(0.0, current_search_rotation_speed_rad );
		
		status = STATUS_SEARCHING;	
	}
	
	void Follower::enter_SEARCHING_FROM_TRACKING(){
		ROS_INFO("%s[STATUS_TRACKING -> STATUS_SEARCHING]%s",col.MAGENTA,col.END);
		
		speech_coded_norepeat("speech_f_perso");
		lastDetection.ts = ros::Time::now();
		node_current_frequency = 0.9*node_rate_hz;
		current_search_rotation_speed_rad = (float)searchRotationSpeed * searchRotationDir;
		CMDVEL(0.0, current_search_rotation_speed_rad );
		
		status = STATUS_SEARCHING;					
	}
	
	void Follower::enter_TRACKING_FROM_SEARCHING(){
		ROS_INFO("%s[STATUS_SEARCHING -> STATUS_TRACKING]%s",col.MAGENTA,col.END);
		node_current_frequency = node_rate_hz;
		
		speech_coded_norepeat("speech_f_visto");
		
		status = STATUS_TRACKING;
	}
	
	void Follower::enter_TRACKING_FROM_IDLE(){
		ROS_INFO("%s[STATUS_IDLE -> STATUS_TRACKING]%s",col.MAGENTA,col.END);
		node_current_frequency = node_rate_hz;
		
		speech_coded_norepeat("speech_f_visto");
		
		status = STATUS_TRACKING;
	}

	void Follower::enter__STATUS_MANAGER(){
		ros::Time time_now = ros::Time::now();

		switch (status) {
			case STATUS_WAIT_FOR_START:
				time_previous_status = ros::Time::now();
				//attende il callback di start
				ROS_INFO_ONCE( "%s Please publish [/start_followme=true] to begin %s",col.YELLOW,col.END) ;
				node_current_frequency =2.0;
				break;

			case STATUS_IDLE:
				if (pir)  {
					if (time_now >	time_previous_status + ros::Duration(4)){
						time_previous_status = ros::Time::now();							
						enter_SEARCHING_FROM_IDLE();
					}					
				}			 
				break;

			case STATUS_SEARCHING:
							
				ROS_INFO_THROTTLE(1, "[STATUS_SEARCHING] rotation at %f rad/s", current_search_rotation_speed_rad);
				

				if (isPersonDetected) //messo a true dal cbk_detectedObjects
				{
					isPersonDetected =false;
					time_previous_status = ros::Time::now();	
					enter_TRACKING_FROM_SEARCHING();
				}else
				{
					// se ho perso la detection da troppo tempo...
					if (time_now >	lastDetection.ts + ros::Duration(lastdetection2stop_sec)) {
						enter_IDLE_FROM_SEARCHING();		
					}
				}
				break;


			case STATUS_TRACKING:
				// se ho perso la detection da piu di lastDetection2search_sec ...
				if (time_now >  (lastDetection.ts + ros::Duration(lastDetection2search_sec))) {
				
					time_previous_status = ros::Time::now();								
					enter_SEARCHING_FROM_TRACKING();
				} 				
				break;



			default:
				break;
		}
		
	
	}

/////////////////////////////////////////
/*
 * Converte la laghezza w in pixel in metri di distanza approssimata da percorrrere in avanti
 * in :w = dimensione oggetto in pixel
 * out: distanza in cm in corrispondenza della colonna del picco luminoso
 * */
float Follower::w2dist(int w, int imgW) {
	const float e = 2.71828182845904;
	float d = 0.0; // distanza stimata
	//float w1metro = 90.0 ; //ampiezza in pixel di una figura umana a distanza di un metro
/*	
	if      (w <  80) { d=1.8;  }
	else if (w < 100) {	d=1.5;	}
	else if (w < 130) {	d=1.0;	}
	else if (w < 200) {	d=0.5;	}
	else d =0.1;*/
	
	//{
	d=  0.1876572 + (3.67639 - 0.1876572)/(1 +  pow((w/82.42017) , 2.616393)  );
	//}
	printf("\n%s Dist: %fm   %s", col.RED, d, col.END);
	return d;
}



/////////////////////////////////////////
// T R A C K
/////////////////////////////////////////
// chiamato dal callback della detection

//usa lastDetection
void Follower::track(detect_t target) {

	// Stimo distanza e angolo
	float estimatedDist= w2dist(target.width, imageW); // >0 significa a Destra; <0 SX
	if (target.distanceFromCenter > 0 ){
		searchRotationDir = -1; // CW , verso destra
	}
	else{
		searchRotationDir = 1;	// CCW , verso sinistra
	};


	// PERSONA NELLA BANDA CENTRALE ?
	if (fabs(target.distanceFromCenter) <= thresholdCenter) {// persona al centro	-> non ruoto
    
			//--------------------------------------------------
			// stabilisco la velocità lineare
			//--------------------------------------------------
			if (target.width  > tooNearFactor * (float)imageW) { 
			
				if (!debug_onlyRotation)
				{
					moveToTargetPose(-0.20,0.0);//CMDVEL(speed, 0.0); //			
				}				
				
				printf( "\n%s TRACK: Detection al centro ma troppo vicino: %f [thresholdCenter: %d ] %s"  , col.YELLOW, target.distanceFromCenter,thresholdCenter, col.END);		 
				speech_coded_norepeat("speech_f_vicino");

			} else if (target.width < tooFarFactor * imageW) { // troppo lontano?
 				if (!debug_onlyRotation)
				{					
					moveToTargetPose(estimatedDist ,0.0);//CMDVEL(trackLinerarSpeed, 0.0);//
				}
				printf("\n %s TRACK: Detection al centro ma lontano Fw dist %f m %s", col.GREEN, estimatedDist, col.END);				 
				speech_coded_norepeat("speech_f_lontano");

			} else { // ok centrato

				STOP;
				printf("\n %s TRACK: RAGGIUNTA PROSSIMITA' ALLA DISTANZA DI: %f m %s", col.MAGENTA, estimatedDist, col.END);
				speech_coded_norepeat("speech_f_giusto");
			}

	} 
	else //distanza oltre banda centrale ->ruoto
	{
	  
		// < 0  è a destra
		float estimated_angle_deg = pixel2deg_H(target.distanceFromCenter , cameraHFOV, imageW);
		moveToTargetPose(0.0, estimated_angle_deg);
		printf("\n %s TRACK ROTATE: pixel dist: %f >>>> angle_deg : %f  %s",col.RED,target.distanceFromCenter, estimated_angle_deg, col.END); 

	}
	float dist_y = 0;
	publish_Marker(estimatedDist,dist_y);
}


void Follower::cbk_doppler(const std_msgs::Bool &msg) {
  if (pir != msg.data) {
    pir = msg.data;
    printf("\nDOPPLER changed to %s", msg.data?"True":"False");
  }
}


////////////////////////////////////////////////////////////////////////////
// Vengono calcolati:
// a) le coordinate di  detectedPerson (float32!)
// b) messo isPersonDetected a true se è detettata una persona
/////////////////////////////////////////////////////////////////////////////
void Follower::cbk_detectedObjects(  const dnn_detect::DetectedObjectArray &detectionArray) {
	dnn_detect::DetectedObject obj; // oggetto corrente
	int detectionCenterX = 0;       //= ( (float)x_min +(float)x_max ) / 2;
	int detectionSize = 0;          //= (float)(x_max - x_min);
	
	
	// Verifico se le detections contengono almeno  una persona
	// individua la persona da seguire -se presente- tra le varie detections
	float_t x_min, x_max, y_min, y_max;
	for (int i = 0; i < detectionArray.objects.size(); i++) {
		obj = detectionArray.objects[i];
		if ((obj.class_name.compare("person")==0) ||  (obj.class_name.compare("persona")==0) ){
			consecutiveDetectionCount +=1;
			isPersonDetected =true;
			lastDetection.ts = ros::Time::now();
			lastDetection.posX=  (obj.x_max + obj.x_min)/2;
			lastDetection.width = (obj.x_max - obj.x_min);
			lastDetection.distanceFromCenter = lastDetection.posX - (float_t)imageW/2;
			if (status== STATUS_TRACKING)
			{
				printf("\n%s Person at posX: %f, centerdist: %f  width: %F  %s", col.GREEN, lastDetection.posX ,lastDetection.distanceFromCenter,  lastDetection.width , col.END);
			}else
			{
				printf("\n%s Person at posX: %f, centerdist: %f  width: %F  %s", col.BOLD, lastDetection.posX ,lastDetection.distanceFromCenter,  lastDetection.width , col.END);				
			}
			/*
			if (obj.confidence > min_confidence) {
				isPersonDetected = true;
				detectionSize =(float)obj.x_max - (float)obj.x_min ;
				* 
				// se la persona è più vicina della detection precedente la memorizzo
				if (detectionSize > detectedPerson.z) {
					
					detectedPerson.x = ((float)obj.x_min + (float)obj.x_max) / 2;
					detectedPerson.y = (obj.y_min + obj.y_max) / 2 ;
					detectedPerson.z = detectionSize;
					ROS_INFO("\t\t person: x[%f,%f] y[%f,%f]", obj.x_min,obj.x_max, obj.y_min , obj.y_max);
				}
				


			} // if confidence
			else {
				ROS_INFO("person confidence level %f below minimum %f",obj.confidence, min_confidence);
			}
			*/
			
				
		} // if person
		else //something not person
		{
			consecutiveDetectionCount = 0;
			//printf("\n detected something not person");
		}
	} // end for each detection

		// calcolo centro
		//detectionCenterX = detectedPerson.x;
		//detectionSize = detectedPerson.z;
	
	//if ((isPersonDetected)& (status != STATUS_WAIT_FOR_START)) {
	if ((consecutiveDetectionCount  >= MIN_CONSECUTIVE_DETECTIONS )& (status != STATUS_WAIT_FOR_START)) {
		// aggiorno lo stato se non è in TRACKING		
		if (status == STATUS_IDLE) {		
			enter_TRACKING_FROM_IDLE();
			
		} else if (status == STATUS_SEARCHING) {		
			enter_TRACKING_FROM_SEARCHING();
		}	

		////////////////////////////////////////////////////
		/// esegue il tracking 
		////////////////////////////////////////////////////
		track(lastDetection);
		////////////////////////////////////////////////////


	} // end isPersonDetected
	
  // se non ci sono persone non faccio nulla
}
void Follower::cbk_detectedPersons(  const dnn_detect::DetectedObjectArray &detectionArray) {
	dnn_detect::DetectedObject obj; // oggetto corrente
	int detectionCenterX = 0;       //= ( (float)x_min +(float)x_max ) / 2;
	int detectionSize = 0;          //= (float)(x_max - x_min);
	
	consecutiveDetectionCount +=1;
	isPersonDetected =true;

	// individua la persona da seguire -se presente- tra le varie detections
	float_t x_min, x_max, y_min, y_max;
	for (int i = 0; i < detectionArray.objects.size(); i++) {
		obj = detectionArray.objects[i];


			lastDetection.ts = ros::Time::now();
			lastDetection.posX=  (obj.x_max + obj.x_min)/2;
			lastDetection.width = (obj.x_max - obj.x_min);
			lastDetection.distanceFromCenter = lastDetection.posX - (float_t)imageW/2;
			if (status== STATUS_TRACKING)
			{
				printf("\n%s Person at posX: %f, centerdist: %f  width: %F  %s", col.GREEN, lastDetection.posX ,lastDetection.distanceFromCenter,  lastDetection.width , col.END);
			}else
			{
				printf("\n%s Person at posX: %f, dist. dal centro: %f  width: %F  %s", col.BOLD, lastDetection.posX ,lastDetection.distanceFromCenter,  lastDetection.width , col.END);				
			}

	} // end for each detection

		// calcolo centro
		//detectionCenterX = detectedPerson.x;
		//detectionSize = detectedPerson.z;
	
	//Evita i falsi positivi 
	if ((consecutiveDetectionCount  >= MIN_CONSECUTIVE_DETECTIONS )& (status != STATUS_WAIT_FOR_START)) {
		// aggiorno lo stato se non è in TRACKING		
		if (status == STATUS_IDLE) {		
			enter_TRACKING_FROM_IDLE();
			
		} else if (status == STATUS_SEARCHING) {		
			enter_TRACKING_FROM_SEARCHING();
		}	

		////////////////////////////////////////////////////
		/// esegue il tracking 
		////////////////////////////////////////////////////
		track(lastDetection);
		////////////////////////////////////////////////////


	} // end
	

}

/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// SPEECH 	
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
void Follower::speech_coded(const std::string text){
	msg_speech_coded.data =text;
	pub_speech_coded.publish(msg_speech_coded);
}

void Follower::speech_coded_norepeat(const std::string text){
	if(! text.compare(lastSpeech) == 0) {
		speech_coded(text); 
		lastSpeech=text; 
	}
}

// pubblica la posizione stimata delle persona che sta seguendo
void Follower::publish_Marker(float x, float y){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "robot";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	pub_person_marker.publish( marker );	
}



void Follower::cbk_start_followme(const std_msgs::Bool &msg){
	if (msg.data)
	{
		enter_IDLE_FROM_START(); // solo ROS_INFO

	}else{
		ROS_INFO(" RICEVUTO STOP FOLLOW ME. MI FERMO");
		speech_coded_norepeat("speech_f_stop");
		start_followme= false;
		STOP;
		status = STATUS_WAIT_FOR_START;
	}

}

////////////////////////////////////////////////////
/// Entry point del nodo
////////////////////////////////////////////////////
Follower::Follower(ros::NodeHandle &nh) {

	// -------------------------------------------------------------------------
	//  parametri 
	// -------------------------------------------------------------------------
	nh.param<float>("node_rate", node_rate_hz, 10);
	
	nh.param<std::string>("cmdvel_topic", cmdvel_topic, "/cmd_vel");
	nh.param<std::string>("camerainfo_topic", camerainfo_topic, "/robot/usbcam");
	nh.param<float>("width2distance", width2distance, 0.007843f);
	nh.param<float>("trackRotationSpeed", trackRotationSpeed, 1.3);
	nh.param<float>("searchRotationSpeed", searchRotationSpeed, 1.0);
	nh.param<float>("trackLinerarSpeed", trackLinerarSpeed, 0.2);
	nh.param<int>("thresholdCenter", thresholdCenter, 30);
	nh.param<float>("tooNearFactor", tooNearFactor, 0.7);
	nh.param<float>("tooFarFactor", tooFarFactor, 0.4);


	nh.param<float>("cameraHFOV", cameraHFOV, 1.570796327);
	nh.param<int>("lastDetection2search_sec", lastDetection2search_sec, 5);
	nh.param<int>("lastdetection2stop_sec", lastdetection2stop_sec,
				10); // secondi per fermarsi ed entrare in hidle
	nh.param<float>("min_confidence", min_confidence, 0.4);
	nh.param<bool>("debug_onlyRotation", debug_onlyRotation, true);
	nh.param<bool>("start_followme", start_followme, false);
	
	int servocamera_startupangle_deg;
	nh.param<int>("servocamera_startupangle_deg", servocamera_startupangle_deg, 90);

	
	// -------------------------------------------------------------------------

	if (debug_onlyRotation) {
		trackLinerarSpeed=0.0;
	}


	// -------------------------------------------------------------------------
	// subscriptions
	// -------------------------------------------------------------------------
	sub_detectedPersons =nh.subscribe("/dnn_persons", 1, &Follower::cbk_detectedPersons, this,
				   			ros::TransportHints().tcpNoDelay()); // richiede #include <transport_hints.h>
	sub_doppler = nh.subscribe("/doppler", 1, &Follower::cbk_doppler, this,	ros::TransportHints().tcpNoDelay());

	sub_camerainfo = nh.subscribe(camerainfo_topic, 1, &Follower::cbk_camerainfo, this);

	sub_start_followme = nh.subscribe("/start_followme", 1,&Follower::cbk_start_followme,this);
	// -------------------------------------------------------------------------
	
	
	// -------------------------------------------------------------------------
	// pubblications
	// -------------------------------------------------------------------------
	pub_cmdvel = nh.advertise<geometry_msgs::Twist>(cmdvel_topic, 1);

	pub_chatter = nh.advertise<std_msgs::String>("/chatter", 1); // mia aggiunta
	pub_speech_once = nh.advertise<std_msgs::String>("/speech_once", 1); // mia aggiunta
	pub_speech_coded = nh.advertise<std_msgs::String>("/speech_coded", 1); // mia aggiunta
	
	std_msgs::Int16 raspicam_tilt_demand;	// imposto l'angolo migliore per vedere le persone da vicino
	pub_raspicamtilt = nh.advertise<std_msgs::Int16>("/servo_raspicam", 1);
	
	pub_faretto = nh.advertise<std_msgs::Bool>("/faretto", 1);
	
	raspicam_tilt_demand.data = servocamera_startupangle_deg; 
	pub_raspicamtilt.publish(raspicam_tilt_demand);
	
	pub_target_pose = nh.advertise<geometry_msgs::Twist>("/target_pose",1);

	pub_person_marker = nh.advertise<visualization_msgs::Marker>( "person_marker", 0 );	
	
	ros::spinOnce();
	/*
	// ACTION_LIB CLIENT --------------------------

	  // crea il action client
	  // passo nome server e  true -> causes the client to spin its own thread
	  //
	  //tell the action client that we want to spin a thread by default
	  // http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
	  MoveBaseClient ac("move_base", true);
	 //actionlib::SimpleActionClient<rm_follower::MoveAction>
	 ac("move_action_server", true);


	  // wait for the action server to start
	  ROS_INFO("Waiting for action server to start.");
	  ac.waitForServer(); //will wait for infinite time
	  ROS_INFO("Action server started.");
	  //----------------------------------------------

	  // goal di test---------------------------
	  move_base_msgs::MoveBaseGoal goal; //rm_follower::MoveAction goal;

	  //we'll send a goal to the robot to move 1 meter forward
	  goal.target_pose.header.frame_id = "base_link";
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = 0.05;
	  goal.target_pose.pose.orientation.w = 1.0;


	  // invio goal
	  ac.sendGoal(goal);
	  // attendo risultato
	  ac.waitForResult();//ac.waitForResult(ros::Duration(5.0));
	  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		  printf("Yay! Goal raggiunto");
	  printf("Current State: %s\n", ac.getState().toString().c_str());
	  //------------------------------------------------------------
	*/



	// -------------------------------------------------------------------------
	// inizializzazioni
	// -------------------------------------------------------------------------	

	
	// inizializzo ad un tempo molto prima di lastdetection2stop_sec prima di ora
	lastDetection.ts =  ros::Time::now() - ros::Duration(2 * lastdetection2stop_sec);

	
	status = STATUS_WAIT_FOR_START;
	
	float node_current_frequency = 2.0;
	ros::Rate node_rate(node_current_frequency) ;
	// -------------------------------------------------------------------------
	// loop principale
	// -------------------------------------------------------------------------	
	while (ros::ok()) {
		// ros::Duration updateInterval = time_now -lastDetection.ts;
		enter__STATUS_MANAGER();
		
		ros::spinOnce();	
		node_rate = ros::Rate(node_current_frequency);
		node_rate.sleep();
		//ros::Duration(node_loop_interval_sec).sleep() ; // Sleep for n seconds    //rate.sleep();
	}
	ROS_INFO("----STOPPING----");
	STOP;
	ROS_INFO("----END FOLLOWER----");
	
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "rm_follower");
  ros::NodeHandle nh("~");

  Follower node = Follower(nh);

  return 0;
}
