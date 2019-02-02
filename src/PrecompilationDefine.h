#pragma once

// Contient les defines de Precompilation pour activer d�sactiver certaines parties du code avant compilation.
// mettre en commentaire pour supprimer un bloc de code( ne sera pas compil�)

// Attention � la version release FINALE !!! 
#define PRECOMPIL_MEASURE_MINIATURE							// Si il est d�fini, switch sur les valeurs miniatures des "Real Y"
															// ( hauteurs r�elles mesur�es et connues comme la hauteur de la camera et des vision target )
															//
//#define PRECOMPIL_IMAGEFILTER_HSV							// Si il est d�fini, active le traitement HSV de la vid�o pour extraire les visiontarget
															//
#define PRECOMPIL_IMAGEFILTER_CANNY_AND_THRESHOLD			// Si il est d�fini, active le traitement CANNY et THRESHOLD de la vid�o pour extraire les visiontarget

// Filtres logiques utilis�s pour d�cider si une paire de VisionTarget pr�sum�es est valide ( permet d'�liminer les vision target parasites qui n'en sont pas )
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_VISIONTARGETS_ANGLE_TOO_LARGE
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_HIGH_RATIO
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_QUADSIDE_RATIO
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_ANGLE_QUADTOP_HORIZONTAL
#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_ANGLE_QUADBASE_HORIZONTAL