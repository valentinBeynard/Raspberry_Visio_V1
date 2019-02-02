#include "VisionTargetPair.h"
#include "Tools.h"
#include "Values.h"

const float VisionTargetPair::assemble(VisionTarget * pshapeA, VisionTarget * pshapeB)
{
	Point2f topv, basev;
	float f, basenorm,topnorm;

	// setup
	m_pvisionTargetA = pshapeA;
	m_pvisionTargetB = pshapeB;

	m_quality = 0.0f;


	float nb = 0.0f; // nombre de notes attribu�es.
	// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Angle apparent entre les deux shapes:
	// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// 1: L'angle est-il "bien" ou "mal" orient� ?
	//		X product VT1.n1 x VT2.n1 < 0 signifie que l'angle entre les deux visions target est 'bien' orient� ( l'angle entre les deux vecteurs est ouvert en bas )
	//		X product VT1.n1 x VT2.n1 > 0 signifie que l'angle entre les deux visions target est 'mal' orient� ( l'angle entre les deux vecteurs est ouvert en haut/ferm� en bas )
	//
	//		 .....		.....						.....				 .....	
	//		/ 	/		 \   \						 \   \				/ 	/	
	//	   /   /		  \   \						  \   \			   /   /	
	//    /   /			   \   \					   \   \		  /   /		
	//	 /   /			    \   \					    \   \		 /   /		
	//  .....			     .....					     .....		.....		
	//
	//			Bien orient�							   Mal orient�	
	if (m_pvisionTargetA->n01.cross(m_pvisionTargetB->n01) > 0.0f)
	{
		// Angle mal orient�
		m_quality = -1.0f;
		return -1.0f;
	}
	else
	{
		// Angle Bien Orient�:
		m_quality += 1.0f;
		nb += 1.0f;
	}

#ifdef PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_VISIONTARGETS_ANGLE_TOO_LARGE	

	// 2: L'angle est-il a "peu pr�s" bon ? C'est � dire ne semble t'il pas "TROP grand" entre les deux Vision target?
	//		Dot Product,  VT1.n1 � VT2.n1 < limit signifie que l'angle est trop grand.
	//		rappel: 	
	//				Les deux vecteurs m_pvisionTargetA->n01 et m_pvisionTargetB->n01 sont normalis� ( longeur = 1 ) donc leur produit scalaire est �gal au cosinus de l'angle entre eux.
	//				-Dot product =  1 signifie que les deux vecteurs sont align�s (angle = 0)et regardent dans la m^zmz direction
	//				-Dot product =  0 signifie que les deux vecteur forment un angle de 90�.
	//				-Dot product = -1 signifie que les deux vecteur  sont align�s (angle = 180�) mais oppos�s
	//				-Plus l'angle est ferm� et plus le dot product se rapproche de 1, plus l'angle est ouvert et plus il se rapproche de -1.

	//				L'angle r�el entre les deux vision target est d'environ 14.5 deg ( cos(14.5 deg) = 0.96814 ), mais une fois projet� ( vue perspective ) il pourra sembl� plus �troit ( angle < 14.5) , peut �tre m�me l�g�rement plus grand dans certain cas ( angle > 14.5 )... 
	//				pour un angle de 30� entre les deux vision target ( ce qui correspond environ au double de l'angle r�el) on obtient un Dot Product de 0.8660f ( cos(30 deg) = 0.8660 ... )
	f = m_pvisionTargetA->n01.dot(m_pvisionTargetB->n01);
	if (f < VISIONTARGET1_DOT_VISIONTARGET2_UNDERLIMIT)
	{
		// Angle trop grand
		m_quality = -1.0f;
		return -1.0f;
	}
	else
	{
		// Angle apparement correct:
		// Comparons le cosinus de cet angle avec le cosinus Etalon, c'est � dire celui d'un couple de Vision target pleinement de face. ( cos( 14.5deg ) = 0.96814 .
		// l'objectif, attribuer la note de 1 si l'angle est �gal � celui de l'�talon et d�gressive jusqu'� la limite base ( note = 0.0f ).
		// Notez que la note sera �galement d�gressive si l'angle est plus grand que l'angle Etalon. 
		f = fabs( f - VISIONTARGET1_DOT_VISIONTARGET2_STANDARD);
		m_quality += 1.0f - f / (VISIONTARGET1_DOT_VISIONTARGET2_STANDARD - VISIONTARGET1_DOT_VISIONTARGET2_UNDERLIMIT);
		nb += 1.0f;
	}
#endif
#ifdef PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_HIGH_RATIO
	// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Rapports remarquables
	// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Rapport des "hauteurs"
	// Les deux visions targets sont identiques, � l'exception de leur orientation (symetrique / l'axe verticale ). 
	// Une fois projet�es, les deux visions target d'un m�me couple (suppos�) auront certes une l�g�re difference de taille ( la VT la plus pr�s de la camera apparaitra plus grosse) mais cette diff�rence
	// reste "l�g�re", particuli�rement sur la "hauteur". Donc une trop grande diff�rence de hauteur entre deux VT d'un m�me couple suppos� peut �tre un bon indicateur pour invalider ce couple.
	f = MIN(m_pvisionTargetA->norm01, m_pvisionTargetB->norm01) / MAX(m_pvisionTargetA->norm01, m_pvisionTargetB->norm01); // f �volue entre 0 et 1: 0 < f < 1
	if( f <  VISIONTARGET_COUPLE_VTA_VTB_HEIGHT_RATIO_UNDERLIMIT )
	{
		// Trop grande diff�rence de taille entre les deux vision Target
		m_quality = -1.0f;
		return -1.0f;
	}
	else
	{
		// 'hauteur' relative apparement  correcte:
		// Comparons le rapport de hauteur calcul� avec le rapport �talon c'est � dire celui d'un couple de Vision target pleinement de face.
		// Dans ce cas, les deux vision target ont exactement la m�me hauteur,leur rapport vaut 1.0f.
		// Plus les deux hauteurs ont des longueurs similaires , meilleure sera la note.
		m_quality += f;
		nb += 1.0f;
	}
#endif
#ifdef PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_QUADSIDE_RATIO
	// Rapport de la Vision Target avec la taille apparente la plus petite sur la longueur apparente du top du Quadrilatere du couple. 
	// Les deux vision Target d'un m�me couple sont toujours s�par�es par la m�me distance. Les Visual Targets �tant inclin�es l'une vers l'autre, 
	// la distance s�parant les points les plus hauts est plus longue que celle s�parant les points les plus bas. 
	// Une fois projet�es, les distances apparentes diff�rent mais leurs rapports demeurent globalement coh�rent. 
	topv = m_pvisionTargetB->m_higher2D - m_pvisionTargetA->m_higher2D;
	topnorm = sqrtf(topv.x*topv.x + topv.y*topv.y);
	if ( (topnorm < VISIONTARGET_COUPLE_QUADSIDE_NORM_MIN ) || ((f = MIN(m_pvisionTargetA->norm01, m_pvisionTargetB->norm01)/topnorm) < VISIONTARGET_COUPLE_QUADSIDE_RATIO_UNDERLIMIT))
	{
		// Proportion du quadrilat�re non conforme. C'est � dire quadrilat�re trop �tir� horizontalement
		m_quality = -1.0f;
		return -1.0f;
	}
	else
	{
		// On compare le ratio r avec le ratio �talon c'est � dire le ratio obtenu avec un quadrilatere pleinement de face.
		// Dans ce cas, le ratio est de 0.4929.
		// Empiriquement on constate que ce ratio �volue entre environ 0.4 au minimum et jusqu'� environ 5 au maximum.
		// On attribuera la note maximum � un quadrilat�re ayant des proportions identiques au quadrilat�re Etalon.
		f = 1.0f - MIN( 1.0f, fabs(f - VISIONTARGET_COUPLE_QUADSIDE_RATIO_STANDARD) / (VISIONTARGET_COUPLE_QUADSIDE_RATIO_STANDARD - VISIONTARGET_COUPLE_QUADSIDE_RATIO_UNDERLIMIT) );
		m_quality += 1.0f;
		nb += 1.0f;
	}
#endif
#ifdef PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_ANGLE_QUADTOP_HORIZONTAL
	// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Angle apparent du couple avec "l'horizontale"
	// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Les visions target sont toutes situ�es dans deux plans horizontaux ( toutes celles du bas dans un "plan bas" et celles du haut des fus�es dans "plan haut" .)
	// Une fois projet�es les Paires de Visions Target semblent toutes align�es avec l'horizontale plut�t qu'avec la verticale. En cons�quence les Couples de Vision Target ayant une 
	// trop forte pente apparente avec l'horizontale peuvent �tre rejet�s.
	// Pour estimer cette pente on regarde simplement l'ordonn�e (valeur absolue) du vecteur directeur de la base du quadrilat�re. Cette valeur repr�sente le sinus de l'angle de la base du quadrilat�re
	// avec l'horizontale.

#ifdef PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_QUADSIDE_RATIO
	// ( ... On r�utilise "topv" et "topnorm" d�j� calcul�s quelques lilgnes plus haut )
	//basev.x /= basenorm; !!! Inutile de normaliser basev.x, car on ne l'utilise pas.
	topv.y /= topnorm;
#else
	// Le bloc de code "PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_QUADSIDE_RATIO" est inactif, on calcule topv et topnorm
	topv = m_pvisionTargetB->m_higher2D - m_pvisionTargetA->m_higher2D;
	topnorm = sqrtf(topv.x*topv.x + topv.y*topv.y);
	//topv.x /= basenorm; !!! Inutile de normaliser basev.x, car on ne l'utilise pas.
	topv.y /= topnorm;
#endif

	if ( fabs(topv.y) > VISIONTARGET_COUPLE_INCLINE_UPPERLIMIT)
	{
		// Proportion du quadrilat�re non conforme. C'est � dire quadrilat�re trop �tir� horizontalement
		m_quality = -1.0f;
		return -1.0f;
	}
	else
	{
		// plus un quadrilat�re est horizontal mieux c'est. Donc plus la valeur absolue de topv.y est proche de 0, meilleure sera la qualit� !  
		m_quality += 1.0f- fabs(topv.y);
		nb += 1.0f;
	}
#endif
#ifdef PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_ANGLE_QUADBASE_HORIZONTAL
	// On refait la m�me chose avec la 'base' du quadrilat�re. En effet, certain "faux couple" form�s par des "fausses Visual Targets" peuvent �tre �tonnement d�form�s. Cette double v�rification peut aider � en �liminer 
	// certains.
	basev = m_pvisionTargetB->m_lower2D - m_pvisionTargetA->m_lower2D;
	basenorm = sqrtf(basev.x*basev.x + basev.y + basev.y);
	//basev.x /= topnorm; !!! Inutile de normaliser basev.x, car on ne l'utilise pas.
	basev.y /= basenorm;
	if (fabs(basev.y) > VISIONTARGET_COUPLE_INCLINE_UPPERLIMIT)
	{
		// Proportion du quadrilat�re non conforme. C'est � dire quadrilat�re trop �tir� horizontalement
		m_quality = -1.0f;
		return -1.0f;
	}
	else
	{
		// plus un quadrilat�re est horizontal mieux c'est. Donc plus la valeur absolue de basev.y est proche de 0, meilleure sera la qualit� !  
		m_quality += 1.0f - fabs(basev.y);
		nb += 1.0f;
	}
#endif


	// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Distance apparente entre les deux shapes
	// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
	cv::Point3f v = m_pvisionTargetB->middleBottom3D - m_pvisionTargetA->middleBottom3D;
	m_norm3DBottom = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
	if (m_norm3DBottom <= FLT_EPSILON_NORM)
	{
		m_quality = -1.0f;
		return -1.0f;
	}
	v = m_pvisionTargetB->middleTop3D - m_pvisionTargetA->middleTop3D;
	m_norm3DTop = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

	float f = m_norm3DTop / m_norm3DBottom;
	float r = MIN(f, PAIR_NORM3DRATIO) / MAX(f, PAIR_NORM3DRATIO);
	
	if (f<0.5f || f>2.0f)
	{
		m_quality = -1.0f;
		return -1.0f;
	}
	else
	{
		m_quality += f;
		nb += 1.0f;
	}
*/	
	
	// Setup Finalisation
	// Calcul du centre du trapeze[A3][A0][B1][B2]
	Tools::SegXSeg(pshapeA->m_higher2D, pshapeB->m_lower2D, pshapeA->m_lower2D, pshapeB->m_higher2D, m_center[VisionTargetPair::TRAPEZOID]);
	
	// Calcul du projet� (selon l'axe Y ) du centre du trap�ze su rle segement [B1][B2], pour obtenir le milieu du segment [B1][B2] en respectant la d�formation perspective
	// ( On suppose donc ici qu'une droite verticale dans le monde reste une droite verticale une fois projet�e ... ( ce qui n'est pas tout a fait vrai ... mais bon )
	basev = m_center[0];
	basev.y += CAPTURED_IMAGE_HEIGHT;
	Tools::SegXSeg(pshapeA->m_lower2D, pshapeB->m_lower2D, m_center[0], basev, m_center[VisionTargetPair::TRAPEZOID_BOTTOM_SIDE]);

	m_quality /= nb;

	return m_quality;
/*
	// Une Paire d�j� existante r�f�rence pshapeA !
	if(pshapeA->m_pVisionTargetPair)
	{
		assert(pshapeB->m_pVisionTargetPair==NULL);

		// La paire pr�-existante semble de moins bonne qualit�.
		// La nouvelle paire la remplace....
		if(pshapeA->m_pVisionTargetPair->m_quality < m_quality)
		{
			
		}
		else // La paire pr�-existante semble de meilleure qualit� il faut la garder ! Et tout simplement oublier la nouvelle.
		{
			m_quality = -2.0f;
			return m_quality;
		}
	}
	else if(pshapeB->m_pVisionTargetPair)// Une Paire d�j� existante r�f�rence pshapeB !
	{
		// La paire pr�-existante semble de moins bonne qualit�.
		// La nouvelle paire la remplace....
		if (pshapeB->m_pVisionTargetPair->m_quality < m_quality)
		{

		}
		else // La paire pr�-existante semble de meilleure qualit� il faut la garder ! Et tout simplement oublier la nouvelle.
		{
			m_quality = -2.0f;
			return m_quality;
		}
	}
*/
	
	return m_quality;
}

void VisionTargetChain::buildChainGraph()
{
	Point3f		u, v;
	float		unormxz, vnormxz;

	// 
	for (std::vector<VisionTarget>::iterator vta = m_visionTargets.begin(); vta != m_visionTargets.end() - 1; vta ++)
	{
		for (std::vector<VisionTarget>::iterator vtb = vta + 1; vtb != m_visionTargets.end(); vtb++)
		{
			// Angle entre les deux VisionTarget:
			//-----------------------------------

			// Distance entre les deux vision targets dans le plan XZ:
			//--------------------------------------------------------
			// Les Points Haut des deux visions targets sont-ils s�par� par une distance suffisement en phase avec la r�alit� ? 
			u = vta->m_higher3D - vtb->m_higher3D;
			unormxz = u.x*u.x + u.y*u.y; // Tout aussi juste ici que la 'vraie' distance ( mais on gagne une racine carr�e :) )
			if ( (unormxz < REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP_DISTMIN) || (unormxz > REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_TOP_DISTMAX) )
				continue;

			// Les Points Bas des deux visions targets sont-ils s�par� par une distance suffisement en phase avec la r�alit� ? 
			v = vta->m_lower3D - vtb->m_lower3D;
			vnormxz = v.x*v.x + v.y*v.y; // Tout aussi juste ici que la 'vraie' distance ( mais on gagne une racine carr�e :) )
			if ( (unormxz < REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMIN) || (unormxz > REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMAX))
				continue;

			// Evaluation et Validation du lien:
			if (vta->m_linkSize < VISIONTARGET_LINKSIZE)
			{
				//vta->m_links[vta->m_linkSize].m_pto = &vtb;
				vta->m_linkSize ++;
			}
		}
	}
}
