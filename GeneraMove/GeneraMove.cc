#include "GeneraMove.h"
#include <OPENR/OSyslog.h>
#include <OPENR/core_macro.h>
#include <OPENR/OPENRAPI.h>

#include "entry.h"
#include "../Motion/MotionInterface.h"
#include <iostream>
#include <fstream>
#include <math.h>


//char* model="ERS7";
typedef unsigned char BYTE;

int grids_x = 4;
int grids_y = 4;
int last_turn_left = 0;
int walks = 0;
int kick_next = 0;





GeneraMove::GeneraMove(){
	sph = 0;
	walk_period = 0;
	imageVec = NULL;
	fbkID = oprimitiveID_UNDEF;
	state = WALKING;
	found_ball = NONE;
	last_head_turn = FRONT;
	strong_turn_rate = NONE;
}


/** Inizializzo le strutture di comunicazione inter-object e apro i
    sensori utilizzati*/
OStatus GeneraMove::DoInit(const OSystemEvent& event){
	OSYSDEBUG(("GeneraMove::DoInit()\n"));

	NEW_ALL_SUBJECT_AND_OBSERVER;
	REGISTER_ALL_ENTRY;
	SET_ALL_READY_AND_NOTIFY_ENTRY;

	//
	// Apro la primitiva di gestione del sensore di tatto posteriore
	//
	OStatus result = OPENR::OpenPrimitive(BACKR7_LOCATOR , &mBackrID);
	if (result != oSUCCESS)
	{
		OSYSLOG1((osyslogERROR, "%s : %s %d",
				"GeneraMove::DoInit()",
				"OPENR::OpenPrimitive() FAILED", result));
	}

	//
	// Apro la primitiva di gestione della telecamera
	//
	result = OPENR::OpenPrimitive(FBK_LOCATOR, &fbkID);
	if (result != oSUCCESS)
	{
		OSYSLOG1((osyslogERROR, "%s : %s %d",
				"GeneraMove::DoInit()",
				"OPENR::OpenPrimitive() FAILED", result));
	}

	SetCameraParameter();

	SetCdtVectorData();

	return oSUCCESS;
}


/** Faccio partire le strutture di gestione della comunicazione
    inter-object.*/
OStatus GeneraMove::DoStart(const OSystemEvent& event){
	OSYSDEBUG(("GeneraMove::DoStart()\n"));

	ENABLE_ALL_SUBJECT;
	ASSERT_READY_TO_ALL_OBSERVER;
	count=0;
	// Accendo i motori (per scrupolo)
	OPENR::SetMotorPower(opowerON);


	Motion::MotionCommand command;
	memset(&command, 0, sizeof(command));

	command.motion_cmd=Motion::MOTION_STAND_NEUTRAL;
	command.head_cmd=Motion::HEAD_LOOKAT;
	command.tail_cmd=Motion::TAIL_NO_CMD;
	command.head_lookat=vector3d(150,0,50);
	command.vx=0;
	command.vy=0;
	command.va=0;
	if (sph ==1){
	  subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	  subject[sbjMotionControl]->NotifyObservers();
	  sph=0;
	}
	//Walk();

	return oSUCCESS;
} 


/** Fermo la comunicazione inter-object.*/
OStatus GeneraMove::DoStop(const OSystemEvent& event){
	OSYSDEBUG(("GeneraMove::DoStop()\n"));

	DISABLE_ALL_SUBJECT;
	DEASSERT_READY_TO_ALL_OBSERVER;
	OPENR::SetMotorPower(opowerOFF);

	return oSUCCESS;
} 


/** Distrutto le strutture di comunicazione inter-object.*/
OStatus GeneraMove::DoDestroy(const OSystemEvent& event){
	OSYSDEBUG(("GeneraMove::DoDestroy\()\n"));

	DELETE_ALL_SUBJECT_AND_OBSERVER;

	return oSUCCESS;
}  //DoDestroy() END





/** Funzione invocata al ricevimento di un Assert Ready da parte di
    Motion.*/
void GeneraMove::Ready(const OReadyEvent& event){
  //OSYSDEBUG(("GeneraMove:: ricevuto Assert Ready\n"));
	sph=1;
}

void GeneraMove::GetCamera(const ONotifyEvent& event) {

        imageVec = (OFbkImageVectorData*)event.Data(0);
        OFbkImageInfo* info = imageVec->GetInfo(ofbkimageLAYER_C);
        OFbkImageInfo* infoM = imageVec->GetInfo(ofbkimageLAYER_M);
	byte*          data = imageVec->GetData(ofbkimageLAYER_C);
	byte*          dataM = imageVec->GetData(ofbkimageLAYER_M);

	OFbkImage cdtImage(info, data, ofbkimageBAND_CDT);
	OFbkImage yImage(infoM, dataM, ofbkimageBAND_Y);
	OFbkImage CbImage(infoM, dataM, ofbkimageBAND_Cb);
	OFbkImage CrImage(infoM, dataM, ofbkimageBAND_Cr);

	int width = cdtImage.Width();
	int height = cdtImage.Height();    
	int m = 0;
	int n = 0;
	//int pix_count[grids_x][grids_y];
	int **ball_count = (int**) calloc(grids_x, sizeof(int*));

	int **black_count = (int**) calloc(grids_x, sizeof(int*));
	int **white_count = (int**) calloc(grids_x, sizeof(int*));
	int **grid_matrix = (int**) calloc(grids_x, sizeof(int*));


	 
	for (int i=0; i<grids_x; i++)
	{
		grid_matrix[i] = (int*) calloc(grids_y, sizeof(int));
		black_count[i] = (int*) calloc(grids_y, sizeof(int));
		white_count[i] = (int*) calloc(grids_y, sizeof(int));
		ball_count[i] = (int*) calloc(grids_y, sizeof(int));
	}
	int thrs = 150;
	
	int x, y;
	for (x=0; x < width; x++)
	  {
	    for (y=0; y < height; y++)
	      {
		if (cdtImage.Pixel(x, y) & ocdtCHANNEL0)  // canale del rosa
		  {
		    m = (int) floor( (float) (x * grids_x) / (float) width );
		    n = (int) floor( (float) (y * grids_y) / (float) height );
		    ball_count[m][n]++;
		  }
		if (cdtImage.Pixel(x, y) & ocdtCHANNEL1)  // canale del nero
		  {
		    m = (int) floor( (float) (x * grids_x) / (float) width );
		    n = (int) floor( (float) (y * grids_y) / (float) height );
		    black_count[m][n]++;
		  }
		if (cdtImage.Pixel(x, y) & ocdtCHANNEL2)  // canale del bianco
		  {
		    m = (int) floor( (float) (x * grids_x) / (float) width );
		    n = (int) floor( (float) (y * grids_y) / (float) height );
		    white_count[m][n]++;    
		  }
	      }
	  }


	for (x=0; x < grids_x; x++)
	{
		for (y=0; y < grids_y; y++)
		{
			if (black_count[x][y] > thrs)
			  grid_matrix[x][y] = 0;
			else
			  grid_matrix[x][y] = 100;

			if (ball_count[x][y] > 20)
			  grid_matrix[x][y] = 5;

			if (white_count[x][y] > 380)
			  grid_matrix[x][y] = 50;
		}
	}

	OSYSDEBUG(("grid matrix:\n %d  %d  %d  %d\n %d  %d  %d  %d \n %d  %d  %d  %d\n %d  %d  %d  %d\n\n", 
	grid_matrix[0][0],grid_matrix[0][1],grid_matrix[0][2],grid_matrix[0][3],
	grid_matrix[1][0],grid_matrix[1][1],grid_matrix[1][2],grid_matrix[1][3],	
	grid_matrix[2][0],grid_matrix[2][1],grid_matrix[2][2],grid_matrix[2][3],
	grid_matrix[3][0],grid_matrix[3][1],grid_matrix[3][2],grid_matrix[3][3]));	
	
	OSYSDEBUG(("ball count:\n %d  %d  %d  %d\n %d  %d  %d  %d \n\n", 
	ball_count[2][0],ball_count[2][1],ball_count[2][2],ball_count[2][3],
	ball_count[3][0],ball_count[3][1],ball_count[3][2],ball_count[3][3]));
	
	OSYSDEBUG(("white count:\n %d  %d  %d  %d\n %d  %d  %d  %d \n %d  %d  %d  %d\n %d  %d  %d  %d\n\n", 
	white_count[0][0],white_count[0][1],white_count[0][2],white_count[0][3],
	white_count[1][0],white_count[1][1],white_count[1][2],white_count[1][3],	
	white_count[2][0],white_count[2][1],white_count[2][2],white_count[2][3],
	white_count[3][0],white_count[3][1],white_count[3][2],white_count[3][3]));
	
	Motion::MotionCommand command;
	memset(&command, 0, sizeof(command));

	Motion::MotionCommand searchBall;
	memset(&searchBall, 0, sizeof(searchBall));
	
	
	int ball_grids = 0;
	int *ball_row_matrix = (int*) calloc(grids_y, sizeof(int));

	
	// conto quanti sono i quadrati occupati dalla palla
	// e quante righe sono piene
	for (x=0; x < grids_x; x++)
	  {
	    for (y=0; y < grids_y; y++)
	      {
		if (grid_matrix[x][y] == 5){
		  ball_grids += 1;
		  ball_row_matrix[y] += 1;
		}
		    
	      }
	  }

	// ricerca palla
	if ( (state == SEARCHING_BALL || state == PRE_WALKING ) && ball_grids >= 1){
	  
	  // se la palla è agli estremi
	  if ( ball_row_matrix[0] >= 2 && ball_row_matrix[1] >= 1 &&  ball_row_matrix[2] <= 1 && ball_row_matrix[3] <= 1){
	    strong_turn_rate = LEFT;
	  }
	  else if(ball_row_matrix[3] >= 2 && ball_row_matrix[2] >= 1 &&  ball_row_matrix[1] <= 1 && ball_row_matrix[0] <= 1 ){
	    strong_turn_rate = RIGTH;
	  }
	  found_ball = last_head_turn;
	  state = TRACKING_BALL;
	  last_head_turn = FRONT;
	}
	// inseguimento palla
	else if( state == TRACKING_BALL && ball_grids >= 1){
	  state = KICKING_BALL;
	}
	else if( state == PRE_WALKING ){
	  state = WALKING;
	}


	// guarda a sinistra, poi guarda a destra, poi guarda dritto
	if( state == SEARCHING_BALL){
	  // guardo a sinistra
	  if(last_head_turn == FRONT){
	    command.motion_cmd=Motion::MOTION_STAND_NEUTRAL;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(200,180,50);
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    
	    last_head_turn = LEFT;

	    Wait(static_cast<longword>(500000000));
	  }
	  // guardo a destra
	  else if(last_head_turn == LEFT){
	    command.motion_cmd=Motion::MOTION_STAND_NEUTRAL;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(200,-180,50);
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }

	    last_head_turn = RIGTH;


	    Wait(static_cast<longword>(500000000));
	  }
	  else if(last_head_turn == RIGTH){  
	    command.motion_cmd=Motion::MOTION_STAND_NEUTRAL;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(200,0,50);
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }

	    last_head_turn = FRONT;
	    state = PRE_WALKING;
	    Wait(static_cast<longword>(500000000));
	  }
	}
	
	// 
	//  *   TRACKING  *
	//
	else if ( state == TRACKING_BALL ){
	  if (found_ball == LEFT){
	    found_ball = NONE;
	    command.motion_cmd=Motion::MOTION_WALK_TROT;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(100,0,50);
	    command.vx=100;
	    command.vy=0;

	    if (strong_turn_rate == LEFT)
	      command.va=0.85;
	    else
	      command.va=0.45;

	    strong_turn_rate = NONE;
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    Wait(static_cast<longword>(500000000));
	  }
	  else if (found_ball == RIGTH){
	    found_ball = NONE;
	    command.motion_cmd=Motion::MOTION_WALK_TROT;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(100,0,50);
	    command.vx=100;
	    command.vy=0;


	    if (strong_turn_rate == RIGTH)
	      command.va=-0.85;
	    else
	      command.va=-0.45;

	    strong_turn_rate = NONE;
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    Wait(static_cast<longword>(500000000));
	      }
	  else{
	    command.motion_cmd=Motion::MOTION_WALK_TROT;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(100,0,50);
	    command.vx=100;
	    command.vy=0;

	    if (strong_turn_rate == RIGTH)
	      command.va=-0.15;
	    else if (strong_turn_rate == LEFT)
	      command.va=0.15;
	    else
	      command.va=0.05;

	    strong_turn_rate = NONE;

	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    Wait(static_cast<longword>(500000000));
	      }
	}
	else if (state == KICKING_BALL){
	  
	  if (approaching == 0){
	    state = SEARCHING_BALL;
	    approaching = 1;

	    command.motion_cmd=Motion::MOTION_KICK_FOREWARD;
	    command.head_cmd=Motion::HEAD_NO_CMD;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    Wait(static_cast<longword>(1000000000));
	  }
	  else{
	    approaching = 0;
	    command.motion_cmd=Motion::MOTION_WALK_TROT;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(100,0,50);
	    command.vx=100;
	    command.vy=0;
	    command.va=0.05;
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    Wait(static_cast<longword>(500000000));
	  }
	}
	
	//
	//   *   WALKING  *
	//
	else if ( state == WALKING ){

	  if ( walk_period == 4 ){
	    state = SEARCHING_BALL;
	    walk_period = 0;
	  }
	  
	  walk_period+=1;


	  if(grid_matrix[0][0] == 0 || grid_matrix[0][1] == 0 || grid_matrix[0][2] == 0 || grid_matrix[0][3] == 0){
	    last_turn_left = 1;
	    OSYSDEBUG(("sinistra\n"));
	    command.motion_cmd=Motion::MOTION_WALK_TROT;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(150,0,50);
	    command.vx=0;
	    command.vy=0;
	    command.va=0.65;
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    Wait(static_cast<longword>(1000000000));
	  }
	  else if(grid_matrix[1][1] == 50 || grid_matrix[1][2] == 50 || grid_matrix[2][1] == 50 || grid_matrix[2][2] == 50){
	    OSYSDEBUG(("destra\n"));
	    command.motion_cmd=Motion::MOTION_WALK_TROT;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(150,0,50);
	    command.vx=0;
	    command.vy=0;
	    command.va=-0.55;
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    Wait(static_cast<longword>(1500000000));
	  }
	  else{
	    if (walks >= 3){
	      last_turn_left = 0;
	      walks = 0;
	    }
	    walks += 1;
	    OSYSDEBUG(("dritto\n"));
	    command.motion_cmd=Motion::MOTION_WALK_TROT;
	    command.head_cmd=Motion::HEAD_LOOKAT;
	    command.tail_cmd=Motion::TAIL_NO_CMD;
	    command.head_lookat=vector3d(150,0,50);
	    command.vx=100;
	    command.vy=0;
	    command.va=0.05;
	    if (sph ==1){
	      subject[sbjMotionControl]->SetData(&command,sizeof(Motion::MotionCommand));
	      subject[sbjMotionControl]->NotifyObservers();
	      sph=0;
	    }
	    Wait(static_cast<longword>(500000000));
	  }


	}

	observer[event.ObsIndex()]->AssertReady();
}

/** Funzione che setta i parametri della camera. */
void GeneraMove::SetCameraParameter() {

	OPrimitiveControl_CameraParam shutter(ocamparamSHUTTER_MID);
	OPENR::ControlPrimitive(fbkID, oprmreqCAM_SET_SHUTTER_SPEED, &shutter, sizeof(shutter ) , 0 , 0);

	OPrimitiveControl_CameraParam gain(ocamparamGAIN_HIGH);
	OPENR::ControlPrimitive(fbkID, oprmreqCAM_SET_GAIN, &gain , sizeof(gain), 0,0);

	OPrimitiveControl_CameraParam wb(ocamparamWB_INDOOR_MODE) ;
	OPENR::ControlPrimitive (fbkID,oprmreqCAM_SET_WHITE_BALANCE, &wb, sizeof(wb),0,0);

	// Bilanciamento del bianco automatico
	OPENR::ControlPrimitive (fbkID, oprmreqCAM_AWB_ON, 0, 0, 0, 0);
	// Bilanciamento esposizione automatico
	OPENR::ControlPrimitive (fbkID, oprmreqCAM_AE_ON, 0, 0, 0, 0);

}

void
GeneraMove::SetCdtVectorData()
{
	OStatus result;
	MemoryRegionID  cdtVecID;
	OCdtVectorData* cdtVec;
	OCdtInfo*       cdtPink;
	OCdtInfo*       cdtBlack;
	OCdtInfo*       cdtWhite;

	result = OPENR::NewCdtVectorData(&cdtVecID, &cdtVec);
	if (result != oSUCCESS) {
		OSYSLOG1((osyslogERROR, "%s : %s %d",
				"GeneraMove::SetCdtVectorData()",
				"OPENR::NewCdtVectorData() FAILED", result));
		return;
	}

	// numero di canali da usare
	cdtVec->SetNumData(3);

	// setting della CDT per la palla
	cdtPink = cdtVec->GetInfo(0);
	cdtPink->Init(fbkID, ocdtCHANNEL0);

	//
	// cdtPink->Set(Y_segment, Cr_max,  Cr_min, Cb_max, Cb_min)
	//
	cdtPink->Set( 1, 216, 203, 110, 105);
	cdtPink->Set( 2, 250, 207, 119, 109);
	cdtPink->Set( 3, 240, 238, 120, 114);
	cdtPink->Set( 4, 207, 207, 121, 121);

	cdtPink->Set( 0, 230, 150, 190, 120);
	cdtPink->Set( 5, 230, 150, 190, 120);
	cdtPink->Set( 6, 230, 150, 190, 120);
	cdtPink->Set( 7, 230, 150, 190, 120);
	cdtPink->Set( 8, 230, 150, 190, 120);
	cdtPink->Set( 9, 230, 150, 190, 120);
	cdtPink->Set(10, 230, 150, 190, 120);
	cdtPink->Set(11, 230, 150, 190, 120);
	cdtPink->Set(12, 230, 150, 190, 120);
	cdtPink->Set(13, 230, 150, 190, 120);
	cdtPink->Set(14, 230, 150, 190, 120);
	cdtPink->Set(15, 230, 150, 190, 120);
	cdtPink->Set(16, 230, 150, 190, 120);
	cdtPink->Set(17, 230, 150, 190, 120);
	cdtPink->Set(18, 230, 150, 190, 120);
	cdtPink->Set(19, 230, 150, 190, 120);
	cdtPink->Set(20, 230, 160, 190, 120);
	cdtPink->Set(21, 230, 160, 190, 120);
	cdtPink->Set(22, 230, 160, 190, 120);
	cdtPink->Set(23, 230, 160, 190, 120);
	cdtPink->Set(24, 230, 160, 190, 120);
	cdtPink->Set(25, 230, 160, 190, 120);
	cdtPink->Set(26, 230, 160, 190, 120);
	cdtPink->Set(27, 230, 160, 190, 120);
	cdtPink->Set(28, 230, 160, 190, 120);
	cdtPink->Set(29, 230, 160, 190, 120);
	cdtPink->Set(30, 230, 160, 190, 120);
	cdtPink->Set(31, 230, 160, 190, 120);

	// setting della CDT per il bordo grigio del tracciato
	cdtBlack = cdtVec->GetInfo(1);
	cdtBlack->Init(fbkID, ocdtCHANNEL1);

	//
	// cdtBlack->Set(Y_segment, Cr_max,  Cr_min, Cb_max, Cb_min)
	//
	cdtBlack->Set( 4, 129, 116, 145, 121);
	cdtBlack->Set( 5, 127, 112, 160, 119);
	cdtBlack->Set( 6, 127, 109, 159, 115);
	cdtBlack->Set( 7, 126, 108, 150, 110);
	cdtBlack->Set( 8, 122, 108, 137, 110);
	cdtBlack->Set( 9, 121, 108, 131, 110);
	cdtBlack->Set( 10, 121, 108, 127, 112);
	cdtBlack->Set( 11, 121, 110, 128, 117);


	// setting della CDT per il bordo bianco del tracciato
	cdtWhite = cdtVec->GetInfo(2);
	cdtWhite->Init(fbkID, ocdtCHANNEL2);

	//
	// cdtWhite->Set(Y_segment, Cr_max,  Cr_min, Cb_max, Cb_min)
	//
	cdtWhite->Set( 6, 104, 101, 160, 155);
	cdtWhite->Set( 7, 108, 95, 164, 148);
	cdtWhite->Set( 8, 108, 93, 164, 144);
	cdtWhite->Set( 9, 109, 91, 161, 135);
	cdtWhite->Set( 10, 109, 92, 157, 133);
	cdtWhite->Set( 11, 112, 94, 152, 130);
	cdtWhite->Set( 12, 115, 95, 148, 126);
	cdtWhite->Set( 13, 115, 98, 146, 123);
	cdtWhite->Set( 14, 117, 99, 143, 122);
	cdtWhite->Set( 15, 118, 99, 136, 117);
	cdtWhite->Set( 16, 116, 102, 133, 115);
	cdtWhite->Set( 17, 114, 105, 128, 114);
	cdtWhite->Set( 18, 114, 105, 124, 114);
	cdtWhite->Set( 19, 114, 105, 121, 113);
	cdtWhite->Set( 20, 111, 104, 119, 112);
	


	result = OPENR::SetCdtVectorData(cdtVecID);
	if (result != oSUCCESS) {
		OSYSLOG1((osyslogERROR, "%s : %s %d",
				"GeneraMove::SetCdtVectorData()",
				"OPENR::SetCdtVectorData() FAILED", result));
	}

	result = OPENR::DeleteCdtVectorData(cdtVecID);
	if (result != oSUCCESS) {
		OSYSLOG1((osyslogERROR, "%s : %s %d",
				"GeneraMove::SetCdtVectorData()",
				"OPENR::DeleteCdtVectorData() FAILED", result));
	}
}




