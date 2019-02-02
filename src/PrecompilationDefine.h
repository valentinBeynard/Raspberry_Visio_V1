#pragma once

// Contient les defines de Precompilation pour activer désactiver certaines parties du code avant compilation.
// mettre en commentaire pour supprimer un bloc de code( ne sera pas compilé)

// Attention à la version release FINALE !!! 
#define PRECOMPIL_MEASURE_MINIATURE							// Si il est défini, switch sur les valeurs miniatures des "Real Y"
															// ( hauteurs réelles mesurées et connues comme la hauteur de la camera et des vision target )
															//
//#define PRECOMPIL_IMAGEFILTER_HSV							// Si il est défini, active le traitement HSV de la vidéo pour extraire les visiontarget
															//
#define PRECOMPIL_IMAGEFILTER_CANNY_AND_THRESHOLD			// Si il est défini, active le traitement CANNY et THRESHOLD de la vidéo pour extraire les visiontarget

// Filtres logiques utilisés pour décider si une paire de VisionTarget présumées est valide ( permet d'éliminer les vision target parasites qui n'en sont pas )
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_VISIONTARGETS_ANGLE_TOO_LARGE
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_HIGH_RATIO
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_QUADSIDE_RATIO
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_ANGLE_QUADTOP_HORIZONTAL
#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_ANGLE_QUADBASE_HORIZONTAL