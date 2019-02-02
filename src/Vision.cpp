#include <math.h>
#include <iostream>
#include <fstream>
#include "Vision.h"
#include "Tools.h"
#include "Values.h"




Vision::Vision(const double camerafov_deg, const double cameraYworld, const size_t screenwidth, const size_t screenheight)
{
	m_intrinsic = Mat(3, 3, CV_32FC1);

	/*
	m_planeY[0] = REAL_PLANEY_GROUND;
	m_planeY[1] = REAL_PLANEY_VISIONTARGET_BOTTOM;
	m_planeY[2] = REAL_PLANEY_VISIONTARGET_TOP;
	m_planeY[3] = REAL_PLANEY_VISIONTARGET_CENTER;
	m_planeY[4] = REAL_PLANEY_VISIONTARGET2_BOTTOM;
	m_planeY[5] = REAL_PLANEY_VISIONTARGET2_TOP;
	m_planeY[6] = REAL_PLANEY_VISIONTARGET2_CENTER;
	*/



	m_cameraYworld = cameraYworld;
	
	m_screenWidth = (double)screenwidth;
	m_screenHeight = (double)screenheight;
	m_aspectRatio = (double)screenwidth/(double)screenheight;

	m_fovY = camerafov_deg;
	// pre-computed data 
	m_tangent = tan( NDEGtoRAD(m_fovY * 0.5) );
	m_inverseTangent = 1 / m_tangent;

	m_fovDist = m_screenWidth / (2 * m_tangent);

	/*
	m_Xaxis.x = 1.0f;
	m_Xaxis.y = 0.0f;
	m_Xaxis.z = 0.0f;

	m_Yaxis.x = 0.0f;
	m_Yaxis.y = 1.0f;
	m_Yaxis.z = 0.0f;

	m_Zaxis.x = 0.0f;
	m_Zaxis.y = 0.0f;
	m_Zaxis.z = 1.0f;
	*/
}

Vision::~Vision(){}

// cv::Point3f *res					pointeur sur triplet de float, ou sera stocker les coordonn�es du point. 
// const cv::Point2f *screenpoint	pointeur sur les coordonn�es 2D d'un point � l'�cran. 
// const double planeY				hauteur du plan horizontal dans le "monde". c'est � dire une hauteur par rapport au sol.

void Vision::setFov(const double camerafov_deg)
{
	m_fovY = camerafov_deg;
	// pre-computed data 
	m_tangent = tan(NDEGtoRAD(m_fovY * 0.5));
	m_inverseTangent = 1 / m_tangent;

	m_fovDist = m_screenWidth / (2 * m_tangent);
}

void Vision::setCameraYWorld(const float y)
{
	m_cameraYworld = y;
}

bool Vision::estimate3DPoint(const cv::Point2f& point, const float realYFromTheGround, Point3f &resultat3D)
{
	
	// Test pr�alable, on ne pourra rien faire si le point 2D � une ordonn�e camera nulle ...
	if (fabs(getIntrinsicScreenCenterY() - point.y) <= FLT_EPSILON_2DCOORD)
		return false;

	// Pour faire ces calculs on se base sur les relations simples qui relie les coordonn�es 3Drelatives � la camera aux coordonn�es 2D du point  projet�.
	// X,Y,Z coordonn�es 3D du point A par rapport � la camera.
	// xecran,yecran coordonn�es ecran  de la projection de A.
	// xc,yc coordonn�es 2D de la projection A par rapport au centre ( corrig�) de l'�cran. Le centre de l'�cran coincide avec l'axe Z de la camera ( l'axe de vis�e )
	// Ox,Oy coordonn�es du centre de l'�cran ( corrig�)
	// Fx et Fy Distance focalex et distance focaley (corrig�es)
	// les relation de base sont:
	//								xc = xecran - Ox
	//								yc = Oy - yecran ( et non pas "yecran - Oy" car l'axe Y de l'ecran pointe vers le bas !!! donc on redresse ) 
	//
	//								X/Z = xc/Fx
	//								Y/Z = yc/Fy
	//
	// On connait xc,yc,Fx,Fy et Y !
	// donc on peut tout connaitre :)
	//								d'abord Z � partir de Y 
	//
	//								Z = Y*Fy/yc
	//
	//								puis X � partir de Z
	//
	//								X = Z*xc/Fx
	//
	// C'est parti !
	//
	// On calcul d'abord Y ( Y correspond � la hauteur du point3D PAR RAPPORT A LA CAMERA ).
	// On connait la hauteur r�elle du point par rapport au sol et la hauteur de la camera par rapport au sol...
	// donc c'est facile !:
	resultat3D.y = realYFromTheGround - m_cameraYworld;

	// On retrouve Z a partir de Y ( merci Thales ) et des donn�es Intrinsic de calibrage cam�ra ( distance focale y et centreY corrig� de l'ecran  ). 
	resultat3D.z = (resultat3D.y*getIntrinsicCameraFocalDistY()) / (getIntrinsicScreenCenterY() - point.y );

	// Et enfin X � partir de Z ( merci Thales-bis) et des donn�es Intrinsic de calibrage cam�ra ( distance focale x et centreX corrig� de l'ecran  ). 
	resultat3D.x = (resultat3D.z*(point.x - getIntrinsicScreenCenterX())) / getIntrinsicCameraFocalDistX();

	return true;
}

bool Vision::estimate3DVerticalBipoint(const cv::Point2f(&point)[2], const float(&realYFromTheGround)[2], Point3f(&resultat3D)[2])
{
	// Test pr�alable, on ne pourra rien faire si un des deux point 2D � une ordonn�e camera nulle ...
	if ( (fabs(getIntrinsicScreenCenterY() - point[0].y) <= FLT_EPSILON_2DCOORD) || (fabs(getIntrinsicScreenCenterY() - point[1].y) <= FLT_EPSILON_2DCOORD))
		return false;

	// La d�marche est identique � celle de la fonction "estimate3DPoint" avec un "PLUS" important li� � "verticalit�" r�elle et connue des deux points.
	// "Vertical" signie que leur coordonn�es 3D X et Z seront identiques. 
	// Cette propri�t� va nous permettre de d�tecter un �ventuel angle haut-bas de la camera ( li� � une erreur d'alignement, car la camera est cens�e �tre parfaitement align�e et horizontale ... ;) )
	// ... et surtout de prendre en compte cette erreur dans les calculs. Le r�sultat sera une plus grande pr�cision sur l'estimation des coordonn�es 3D du bipoint et plus particuli�rement du Z !

	// Etape 1) Premiere estimation de Z
	// d'abord les Y ( facile on les connait )
	resultat3D[0].y = realYFromTheGround[0] - m_cameraYworld;
	resultat3D[1].y = realYFromTheGround[1] - m_cameraYworld;
	// On retrouve les Z a partir des Y ( merci Thales ) et des donn�es Intrinsic de calibrage cam�ra ( distance focale y et centreY corrig� de l'ecran  ). 
	resultat3D[0].z = (resultat3D[0].y*getIntrinsicCameraFocalDistY()) / (getIntrinsicScreenCenterY() - point[0].y);
	resultat3D[1].z = (resultat3D[1].y*getIntrinsicCameraFocalDistY()) / (getIntrinsicScreenCenterY() - point[1].y);
	// Le bipoint 3D est suppos� �tre vertical ! les Z sont donc cens�s �tre identiques !
	float erreur = resultat3D[0].z - resultat3D[1].z;

	if ( fabs(erreur) > FLT_TOLERANCE_3DCOORD )
	{
		// En cas d'erreur d�tect�e, on suppose que l'erreur provient d'un mauvais alignement de la camera et on corrige.
		// pour �viter les ralentissement on ne fait q'une seule passe ( pas d'it�rations successives )
		float angDec = asinf(erreur / fabs(resultat3D[0].y - resultat3D[1].y));

		// connaissant l'angle de d�calage on peut maintenant "recaler" en effet les �galit�s:
		//
		//								X/Z = xc/Fx
		//								Y/Z = yc/Fy
		//
		// Ces �galit�s sont en relation avec l'angle vertical et l'angle horizontal ( "camera vers point vis�", "axe de vis�e de la camera" ).
		// Si on appelle l'angle Vertical angV, et l'angle Horizontal angH on a,
		//
		//								X/Z = xc/Fx = Tan(angH)
		//								Y/Z = yc/Fy = Tan(angV)
		//
		// On vient de voir que la camera ne visait pas horizontalement, elle pointe trop haut ou trop bas et on a trouver l'angle vertical de d�calage.
		// Appellons angV0 l'angle mesur� initialement, anvDec l'angle de d�calage vertical de la camera et angV l'angle qu'on aurait du mesurer si la camera �tait parfaitement align�e
		// on a
		//								angV = angV0 - angDec
		//
		// ... On soustrait simplement l'angle de decalage aux angles mesur�s.
		// ( Pour r�cup�rer les angles mesur�s on fait appel a la fonction arctangente )
		// 
		float angV0 = atanf( resultat3D[0].y / resultat3D[0].z ); // = atanf((getIntrinsicScreenCenterY() - point[0].y) / getIntrinsicCameraFocalDistY());
		float angV = angV0 - angDec;

		// Et on applique la m�me logique, mais cette fois avec l'angle corrig� ! 
		//
		//								Y/Z = Tan(angV)
		//								Z	= Y / Tan(angV)
		resultat3D[0].z = resultat3D[0].y / tanf(angV);
		
		// ... m�me chose pour le second point !
		angV0 = atanf(resultat3D[1].y / resultat3D[1].z); // = atanf((getIntrinsicScreenCenterY() - point[1].y) / getIntrinsicCameraFocalDistY());
		angV = angV0 - angDec;
		resultat3D[1].z = resultat3D[1].y / tanf(angV);
	}
	   
	// Et enfin X � partir de Z ( merci Thales-bis) et des donn�es Intrinsic de calibrage cam�ra ( distance focale x et centreX corrig� de l'ecran  ). 
	resultat3D[0].x = (resultat3D[0].z*(point[0].x - getIntrinsicScreenCenterX())) / getIntrinsicCameraFocalDistX();
	resultat3D[1].x = (resultat3D[1].z*(point[1].x - getIntrinsicScreenCenterX())) / getIntrinsicCameraFocalDistX();

	return true;
}





void writePoint3f(Point3f &p, ofstream &fout)
{
	fout << p.x << p.y << p.z << endl;
}

void readPoint3f(Point3f &p, ifstream &fin)
{
	fin >> p.x >> p.y >> p.z;
}

void writeMat(Mat &m, ofstream &fout)
{
	fout << m.rows << " " << m.cols << " " << m.type() << endl;

	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			fout << m.at<double>(i, j) << "\t";
		}
		fout << endl;
	}
}

void readMat(Mat &m, ifstream &fin)
{
	int r, c, t;
	fin >> r >> c >> t;
	m = Mat(r, c,t);

	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			fin >> m.at<double>(i, j);
		}
	}
}


bool Vision::loadCameraIntrinsic(char * filename)
{
	// Fichier cr�� et enregistr� par le programme de calibration de camera.
	ifstream fin(filename, ios::in);  //d�claration du flux et ouverture du fichier
	if (fin)  // si l'ouverture a r�ussi
	{
		size_t vecs_size;
		// Lecture Intrinsic data
		readMat(m_intrinsic, fin);
		readMat(m_distCoeffs, fin);
		
		fin >> vecs_size;
		m_rvecs.resize(vecs_size);
		for (int i = 0; i < m_rvecs.size(); i++)
			readMat(m_rvecs[i], fin);

		fin >> vecs_size;
		m_tvecs.resize(vecs_size);
		for (int i = 0; i < m_tvecs.size(); i++)
			readMat(m_tvecs[i], fin);

		// fermeture du fichier
		fin.close();


		cout << "focal Dist X:" << getIntrinsicCameraFocalDistX() << endl;
		cout << "focal Dist Y:" << getIntrinsicCameraFocalDistY() << endl;
		cout << "Center x:" << getIntrinsicScreenCenterX() << endl;
		cout << "Center y:" << getIntrinsicScreenCenterY() << endl;


		return true;
	}
	else
	{
		// sinon
		cerr << "OPEN FILE ERROR. [ " << filename << " ]" << endl;
		return false;
	}
}

bool Vision::saveCameraIntrinsic(char * filename)
{
	ofstream fout(filename, ios::out | ios::trunc);  //d�claration du flux et ouverture du fichier
	if (fout)  // si l'ouverture a r�ussi
	{
		// Ecriture Intrinsic data
		writeMat(m_intrinsic, fout);
		writeMat(m_distCoeffs, fout);
		fout << m_rvecs.size() << endl;
		for (int i = 0; i < m_rvecs.size(); i++)
			writeMat(m_rvecs[i], fout);
		fout << m_tvecs.size() << endl;
		for (int i = 0; i < m_tvecs.size(); i++)
			writeMat(m_tvecs[i], fout);

		// Ecriture Pied de page
		fout << "# ROBO'LYON CAMERA CALIBRATION DATA -FIRST 2019- #" << endl;

		// instructions
		fout.close();  // on referme le fichier
		return true;
	}
	else
	{
		// sinon
		cerr << "Erreur � l'ouverture !" << endl;
		return false;
	}

}

bool Vision::loadVisionTargetModel(char * filename)
{
	// Fichier cr�� manuellement depuis un export "ASE" depuis 3DSMAX.
	ifstream fin(filename, ios::in);  //d�claration du flux et ouverture du fichier
	if (fin)  // si l'ouverture a r�ussi
	{
		size_t nb;
		
		// First read the number of vertex to read
		fin >> nb;
		
		m_visionTarget3DModel.clear();
		m_visionTarget3DModel.reserve(nb);

		//then read all the value and push them into the vision target 3D model
		Point3f p;
		for (size_t i = 0; i < nb; i++)
		{
			fin >> p.x >> p.y >> p.z;
			m_visionTarget3DModel.push_back(p);
		}
		// fermeture du fichier
		fin.close();
		return true;
	}
	else
	{
		// sinon
		cerr << "OPEN FILE ERROR. [ " << filename << " ]" << endl;
		return false;
	}

}



float Vision::getIntrinsicCameraFocalDistX()
{
	return (float)m_intrinsic.at<double>(0,0);
}

float Vision::getIntrinsicCameraFocalDistY()
{
	return (float)m_intrinsic.at<double>(1, 1);
}

float Vision::getIntrinsicScreenCenterX()
{
	return (float)m_intrinsic.at<double>(0, 2);
}

float Vision::getIntrinsicScreenCenterY()
{
	return (float)m_intrinsic.at<double>(1, 2);
}

float Vision::estimateIntrinsicHorizontalAngleRad(const cv::Point2f & p)
{
	return atan( (p.x - getIntrinsicScreenCenterX()) / getIntrinsicCameraFocalDistX() );
}

float Vision::estimateIntrinsicVerticalAngleRad(const cv::Point2f & p)
{
	return atan( (p.y - getIntrinsicScreenCenterY()) / getIntrinsicCameraFocalDistY() );
}

float Vision::estimateIntrinsicDistanceZ(const cv::Point2f & p, const float realHeightFromTheGround)
{
	// On connait la hauteur de la camera par rapport au sol:
	// ( realHeightFromTheGround - m_cameraYworld ) = Hauteur r�elle par rapport � la cam�ra du point point de l'espace projetant p 

	return ( ( (realHeightFromTheGround - m_cameraYworld)*getIntrinsicCameraFocalDistY() ) / (getIntrinsicScreenCenterY() - p.y) );
}



/// METHODES ANCIENNES / ABANDONNEES *****************************************************************************************************************
///
/*
float Vision::estimateIntrinsicCameraFovXRad()
{
	return atan(m_screenWidth / (2 * (float)m_intrinsic.at<double>(0, 0)));
}

float Vision::estimateIntrinsicCameraFovYRad()
{
	return atan(m_screenHeight / (2 * (float)m_intrinsic.at<double>(1, 1)));
}

bool Vision::get3DPointOnHorizontalPlane(cv::Point3f& res, const cv::Point2f& screenpoint,const size_t plane_index)
{
	double		xl, yl,zl;
	double		x, y;

	double		xc, yc, zc;

	// A/ point de l'�cran --> point 3D en coordonn�es camera: (xc,yc,zc)
	// -----------------------------------------------------------------
	zl = 0.6;//cm
	yl = zl * m_tangent;
	xl = yl * m_aspectRatio;

	x = ((double)screenpoint.x - m_widthoutof2 ) / m_widthoutof2;
	y = (m_heightoutof2 - (double)screenpoint.y ) / m_heightoutof2;

	// 3D point ( on screen) in Camera Coordinates system: xc,yc,zc
	xc = x * xl;
	yc = y * yl;
	zc = zl;

	// B/ INTERSECTION de la demi-droite ( camera, point 3D ) et du plan
	// ------------------------------------------------------------------
	// Donc on a maintenant un rayon qui part de la camera et qui passe par le point de l'�cran qu'on a  situ� en 3D devant la camera...
	// Il ne reste plus qu'a determiner l'intersection de ce rayon avec le plan horizontal situ� � la hauteur "planeY" pass�e en param�tre.
	// !!! Attention cette hauteur est d�finie par rapport au niveau du sol (=0) ou est pos� le robot.

	// 0/ normalisation du vecteur directeur du rayon
	double norm = sqrt(xc * xc + yc * yc + zc * zc);
	xc /= norm;
	yc /= norm;
	zc /= norm;

	// 1/ on calcul le produit scalaire des vecteurs directeurs du rayon et du plan.
	//	celui du rayon est (xc,yc,zc) celui du plan est (0,1,0) car il s'agit d'un plan hoprizontal...
	//		on a donc, dot1	= plandir_x * xc + plandir_y * yc + plandir_z * zc;
	//						= 0 * xc + 1 * yc + 0 * zc
	//						= yc
	double dot1 = yc;

	if (NABS(dot1) <= NF64_EPSILON_VECTOR_DOTPRODUCT) // a la place de = 0, siginie que la droite et le plan sont parall�les ou presque !
	{
		return false;
	}
	else
	{
		// L'id�e est la suivante: ...
		// on fait: u = p - camerapos.
		//				Avec p(0,m_planeY[],0)  point du plan et camerapos(0,m_cameraYWorld,0)) position de la camera
		// on a donc,	ux	= 0 - 0
		//				uy	= m_planeY[plane_index] - m_cameraYworld
		//				uz	= 0 - 0
		//
		// Ensuite, on calcul le produit scalaire du vecteur directeur du plan et de u.
		// soit,	dot2	= 0 * ux  + 1 * uy + 0 * uz
		//					= uy
		//					= (m_planeY[plane_index] - m_cameraYworld)
		//
		// puis on calcul le ratio r des deux produits scalaires.
		//				r	= dot2/dot1
		double r = (m_planeY[plane_index] - m_cameraYworld) / dot1;
		//
		// finalement pour obtenir le point d'intersection on multiplie le vecteur directeur du rayon par r et on ajoute la position de la camera.

		res.x = (float)(xc * r );
		res.y = (float)(yc * r + m_cameraYworld);
		res.z = (float)(zc * r );

		return true;
	}
}
*/