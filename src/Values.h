#pragma once
#include "PrecompilationDefine.h"



// FLAGS
#define FLAG_CAPTURE_PAUSE										0

// Noms de fichiers Externes
#define INTRINSIC_FILENAME							"data/robolyon_intrinsic.txt" //"D:/_PROJETS/FIRST/C++/robolyon_intrinsic.txt"
#define VISIONTARGET_FILENAME						"data/vision_target.txt"		//"D:/_PROJETS/FIRST/C++/vision_target.txt"

// Paramètres des filtres images à l'initialisation
#define FILTRE_CANNY_THRESH1									100
#define FILTRE_CANNY_THRESH2									300
#define FILTRE_THRESHOLD_MIN									150
#define FILTRE_THRESHOLD_MAX									255

#define FILTRE_HLS_HLOW											50		
#define FILTRE_HLS_LLOW											120//70// ou 25 pour voir les détails faiblement lumineux ( VT avec angle approchant les 80 Deg avec l'axe de la camera ... ). 
#define FILTRE_HLS_SLOW											20// ..... MAIS ATTENTION RISQUE de faire apparaitre + de parasite sur le InRangeMask...
#define FILTRE_HLS_HHIGH										95
#define FILTRE_HLS_LHIGH										255
#define FILTRE_HLS_SHIGH										255

// Paramètres Avancés
#define ADVPARAM_HORIZON										100			//  % of screen height
#define ADVPARAM_VISIONTARGET_SIZEMIN							10			//  % of image.width x % image.height
#define ADVPARAM_VISIONTARGET_SIZEMAX							250			//  % of image.width x % image.height


// Mesures réelles


#define REAL_CAMERA_FOV											52.0f		// DEGREE Exact FOV of the camera.



#ifdef PRECOMPIL_MEASURE_MINIATURE
// Hauteurs miniatures ( maquettes ) Mesures sur la maquette N&B #2
#define REAL_Y_GROUND											0.0f		// Ground is level 0
#define REAL_Y_CAMERA											4.20f		// cm par rapport au sol, hauteur réelle de la camera

#define REAL_VISIONTARGET_NORM_01								2.35		// cm
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP				30.156		// cm
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM				27.316		// cm

#define REAL_Y_VISIONTARGETPAIR_CENTER							8.8			// cm par rapport au sol. Point d'intersection des diagonale du VisionTarget QUAD [1],[6],[4],[3] cf "vision_target.txt"
#define REAL_Y_VISIONTARGETPAIR_BOTTOM							7.6			// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond à l'ordonnée des point [4],[3] 
#define REAL_Y_VISIONTARGETPAIR_TOP								10.1		// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond à l'ordonnée des point [1],[6] 

#define REAL_Y_VISIONTARGETPAIR_HCENTER							0			// cm par rapport au sol. Point d'intersection des diagonale du VisionTarget QUAD [1],[6],[4],[3] cf "vision_target.txt"
#define REAL_Y_VISIONTARGETPAIR_HBOTTOM							0			// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond à l'ordonnée des point [4],[3] 
#define REAL_Y_VISIONTARGETPAIR_HTOP							0			// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond à l'ordonnée des point [1],[6] 

// Valeurs limites précalculées à partir des données ci-dessus
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP_DISTMIN		( REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP*REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP*0.5f*0.5f )			//  = dist²
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP_DISTMAX		( REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP*REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP*2.0f*2.0f )			//	= dist²

#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMIN		( REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*0.5f*0.5f )	//  = dist² 
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMAX		( REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*2.0f*2.0f )	//	= dist² 


#else
// Hauteurs réelles 
#define REAL_Y_GROUND											0.0f		// Ground is level 0
#define REAL_Y_CAMERA											??			// cm par rapport au sol, hauteur relle de la camera

#define REAL_VISIONTARGET_NORM_01								14.865		// cm
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP				30.156		// cm
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM				27.316		// cm

#define REAL_Y_VISIONTARGETPAIR_CENTER							??			// cm par rapport au sol. Point d'intersection des diagonales du VisionTarget QUAD [1],[6],[4],[3] cf "vision_target.txt"
#define REAL_Y_VISIONTARGETPAIR_BOTTOM							65.2131		// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond à l'ordonnée des point [4],[3] 
#define REAL_Y_VISIONTARGETPAIR_TOP								80.01		// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond à l'ordonnée des point [1],[6] 

#define REAL_Y_VISIONTARGETPAIR_HCENTER							0			// cm par rapport au sol. Point d'intersection des diagonales du VisionTarget QUAD [1],[6],[4],[3] cf "vision_target.txt"
#define REAL_Y_VISIONTARGETPAIR_HBOTTOM							0			// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond à l'ordonnée des point [4],[3] 
#define REAL_Y_VISIONTARGETPAIR_HTOP							0			// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond à l'ordonnée des point [1],[6] 
#endif






// Vision Targets Recognition


#define VISIONTARGET1_DOT_VISIONTARGET2_STANDARD				0.968147640378107774966f // Cosinus d'un angle de 14.5 degrés.
#define VISIONTARGET1_DOT_VISIONTARGET2_UNDERLIMIT				0.86f		// pour détecter un angle trop important entre les Visions Targets d'un couple.  
																			// pour un angle de 30° entre les deux vision target ( ce qui correspond environ au double de l'angle réel) ...
																			// ...on obtient un Dot Product de 0.8660f ( cos(30 deg) = 0.8660  )

#define VISIONTARGET_COUPLE_VTA_VTB_HEIGHT_RATIO_UNDERLIMIT		0.5f		// Pour détecter les differences de taille (hauteur) trop importantes entre les Visions Targets d'un couple 

#define VISIONTARGET_COUPLE_QUADSIDE_RATIO_STANDARD				(REAL_VISIONTARGET_N01/REAL_VISIONTARGET_NTOP)		// valeur pour un Couple de face ( = 0.4929...)
#define VISIONTARGET_COUPLE_QUADSIDE_RATIO_UNDERLIMIT			0.4f		// Pour détecter les differences de proportion du quadrilatere du couple 
#define VISIONTARGET_COUPLE_QUADSIDE_RATIO_MAX					5.0f		// valeur limite constatée empiriquement (peut-être affinée)
#define VISIONTARGET_COUPLE_QUADSIDE_NORM_MIN					0.0001f		// Longueur minimale d'un côté du quadrilatere du couple, en dessous on considère la longueur comme nulle.	

#define VISIONTARGET_COUPLE_INCLINE_UPPERLIMIT					0.342020143325668f	// Pente maximale de la base et du plafond du quadrilatère du couple.
																					// 0.6f					...	 Sinus d'un angle de 36.869 deg
																					// 0.342020143325668f	... Sinus d'un angle de 20 deg



#define SCALEFACTOR3D			5.0f							// Used by TopView::render For test only

#define CAPTURED_IMAGE_WIDTH	640
#define CAPTURED_IMAGE_HEIGHT	480
#define CAMERA_MODE	6





