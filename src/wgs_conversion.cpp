#include <math.h>

#include <cstdlib>

// WGS84 Parameters
#define WGS84_A		6378137.0		// major axis
#define WGS84_B		6356752.31424518	// minor axis
#define WGS84_F		0.0033528107		// ellipsoid flattening
#define WGS84_E		0.0818191908		// first eccentricity
#define WGS84_EP	0.0820944379		// second eccentricity


// UTM Parameters
#define UTM_K0		0.9996			// scale factor
#define UTM_FE		500000.0		// false easting
#define UTM_FN_N	0.0			// false northing on north hemisphere
#define UTM_FN_S	10000000.0		// false northing on south hemisphere
#define UTM_E2		(WGS84_E*WGS84_E)	// e^2
#define UTM_E4		(UTM_E2*UTM_E2)		// e^4
#define UTM_E6		(UTM_E4*UTM_E2)		// e^6
#define UTM_EP2		(UTM_E2/(1-UTM_E2))	// e'^2

namespace mod_utils {

#include <wgs_conversion.h>

// utility functions to convert geodetic to ECEF positions
void LLH2ECEF(double lat, double lon, double alt, double geo,
	       double& x, double& y, double& z)
{
    double slat = sin(lat* M_PI / 180);
    double clat = cos(lat* M_PI / 180);
    double slon = sin(lon* M_PI / 180);
    double clon = cos(lon* M_PI / 180);

    double height = alt + geo;
    double N = WGS84_A / sqrt(1 - WGS84_E*WGS84_E * slat*slat);

    x = (N + height) * clat * clon;
    y = (N + height) * clat * slon;
    z = ((WGS84_B*WGS84_B) / (WGS84_A*WGS84_A) * N + height) * slat;
};


// utility functions to convert geodetic to UTM positions
void LL2UTM(double lat, double lon, double& x, double& y)
{
    // constants
    const static double m0 = (1 - UTM_E2/4 - 3*UTM_E4/64 - 5*UTM_E6/256);
    const static double m1 = -(3*UTM_E2/8 + 3*UTM_E4/32 + 45*UTM_E6/1024);
    const static double m2 = (15*UTM_E4/256 + 45*UTM_E6/1024);
    const static double m3 = -(35*UTM_E6/3072);

    // compute the central meridian
    int cm = (lon >= 0.0) ? ((int)lon - ((int)lon)%6 + 3) : ((int)lon - ((int)lon)%6 - 3);

    // convert degrees into radians
    double rlat = lat * M_PI/180;
    double rlon = lon * M_PI/180;
    double rlon0 = cm * M_PI/180;

    // compute trigonometric functions
    double slat = sin(rlat);
    double clat = cos(rlat);
    double tlat = tan(rlat);

    // decide the flase northing at origin
    double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

    double T = tlat * tlat;
    double C = UTM_EP2 * clat * clat;
    double A = (rlon - rlon0) * clat;
    double M = WGS84_A * (m0*rlat + m1*sin(2*rlat) + m2*sin(4*rlat) + m3*sin(6*rlat));
    double V = WGS84_A / sqrt(1 - UTM_E2*slat*slat);

    // compute the easting-northing coordinates
    x = UTM_FE + UTM_K0 * V *
	(A + (1-T+C)*pow(A,3)/6 + (5-18*T+T*T+72*C-58*UTM_EP2)*pow(A,5)/120);
    y = fn + UTM_K0 * (M + V * tlat *
	(A*A/2 + (5-T+9*C+4*C*C)*pow(A,4)/24 + (61-58*T+T*T+600*C-330*UTM_EP2)*pow(A,6)/720));
};

//  The below routines convert from X, Y, Z to Lat/Lon/Alt and the reverse.
//  These functions assume the WGS­84 reference system and do not accommodate
//  other datums.
//  For reference, The X, Y, Z coordinate system has origin at the mass center
//  of the Earth, with Z axis along the spin axis of the Earth (+ at North pole).
//  The +X axis emerges from the Earth at the equator­prime meridian intersection.
//  The +Y axis defines a right­hand coordinate system, emerging from the Earth
//  at +90 degrees longitude on the equator.
//
//  The Local Tangential Plane (LTP) coordinates are in latitude/longitude/altitude
//  where North latitude is + and East longitude is +.  Altitude is in meters above
//  the reference ellipsoid (which may be either above or below the local mean sea
//  level).
//  Note: the below code was extracted from working software, but has been edited
//  for this document.  If any problems occur in using this, please contact the
//  engineer who provided it to you for assistance.
//  Define some earth constants
#define MAJA   (6378137.0)             // semi major axis of ref ellipsoid
#define FLAT   (1.0/298.2572235630)    // flattening coef of ref ellipsoid.
//  These are derived values from the above WGS­84 values
#define ESQR   (FLAT * (2.0-FLAT))     // 1st eccentricity squared
#define OMES   (1.0 - ESQR)            // 1 minus eccentricity squared
#define EFOR   (ESQR * ESQR)           // Sqr of the 1st eccentricity squared
#define ASQR   (MAJA * MAJA)           // semi major axis squared = nlmaja**


void ConvertECEFToLTP(double nlecef[3],
                      double * nllat,
                      double * nllon,
                      double * nlalt )
{  //  Convert from ECEF (XYZ) to LTP (lat/lon/alt)
   double     nla0,nla1,nla2,nla3,nla4,nlb0,nlb1,nlb2,nlb3;
   double     nlb,nlc0,nlopqk,nlqk,nlqkc,nlqks,nlf,nlfprm;
   long int   nlk;
   double     ytemp, xtemp;
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   /* b = (x*x + y*y) / semi_major_axis_squared */
   /* c = (z*z) / semi_major_axis_squared       */
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   nlb = (nlecef[0] * nlecef[0] + nlecef[1] * nlecef[1]) / ASQR;
   nlc0 = nlecef[2] * nlecef[2] / ASQR;
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   /* a0 = c * one_minus_eccentricity_sqr       */
   /* a1 = 2 * a0                               */
   /* a2 = a0 + b ­ first_eccentricity_to_fourth*/
   /* a3 = ­2.0 * first_eccentricity_to_fourth  */
   /* a4 = ­ first_eccentricity_to_fourth       */
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   nla0 = OMES * nlc0;
   nla1 = 2.0 * nla0;
   nla2 = nla0 + nlb - EFOR;
   nla3 = - 2.0 * EFOR;
   nla4 = - EFOR;
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   /* b0 = 4 * a0, b1 = 3 * a1                  */
   /* b2 = 2 * a2, b3 = a3                      */
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   nlb0 = 4.0 * nla0;
   nlb1 = 3.0 * nla1;
   nlb2 = 2.0 * nla2;
   nlb3 = nla3;
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   /* Compute First Eccentricity Squared        */
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   nlqk = ESQR;
   for(nlk = 1; nlk <= 3; nlk++)
   {
      nlqks = nlqk * nlqk;
      nlqkc = nlqk * nlqks;
      nlf   = nla0 * nlqks * nlqks + nla1 * nlqkc + nla2 * nlqks +
              nla3 * nlqk + nla4;
      nlfprm= nlb0 * nlqkc + nlb1 * nlqks + nlb2 * nlqk + nlb3;
      nlqk  = nlqk - (nlf / nlfprm);
   }
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   /*
Compute latitude, longitude, altitude  */
   /*­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­­*/
   nlopqk = 1.0 + nlqk;
   if ( nlecef[0] == 0.0 && nlecef[1] == 0.0 )/* on the earth's axis
*/
   {
      /* We are sitting EXACTLY on the earth's axis */
      /* Probably at the center or on or near one of the poles */
      *nllon = 0.0; /* as good as any other value */
		if(nlecef[2] >= 0.0)
         *nllat = M_PI / 2; /* alt above north pole */
		else
         *nllat = - M_PI / 2; /* alt above south pole */
   }
   else
   {
      ytemp = nlopqk * nlecef[2];
      xtemp = sqrt(nlecef[0] * nlecef[0] + nlecef[1] * nlecef[1]);
      *nllat = atan2(ytemp, xtemp);
      *nllon = atan2(nlecef[1], nlecef[0]);
   }
   *nlalt = (1.0 - OMES / ESQR * nlqk) * MAJA *
            sqrt(nlb / (nlopqk * nlopqk) + nlc0);
}  // ConvertECEFToLTP()

void UTMtoLL( const double UTMNorthing, const double UTMEasting, const char* UTMZone,
              double& Lat,  double& Long )
{
//converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
//East Longitudes are positive, West longitudes are negative.
//North latitudes are positive, South latitudes are negative
//Lat and Long are in decimal degrees.
    //Written by Chuck Gantz- chuck.gantz@globalstar.com

    double a = WGS84_A;
    double k0 = UTM_K0;
    double eccSquared = UTM_E2;

    double eccPrimeSquared;
    double e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
    double N1, T1, C1, R1, D, M;
    double LongOrigin;
    double mu, phi1, phi1Rad;
    double x, y;
    int ZoneNumber;
    char* ZoneLetter;
    int NorthernHemisphere; //1 for northern hemispher, 0 for southern

    x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
    y = UTMNorthing;

    ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
    if((*ZoneLetter - 'N') >= 0)
        NorthernHemisphere = 1;//point is in northern hemisphere
    else
    {
        NorthernHemisphere = 0;//point is in southern hemisphere
        y -= 10000000.0;//remove 10,000,000 meter offset used for southern hemisphere
    }

    LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone

    eccPrimeSquared = (eccSquared)/(1-eccSquared);

    M = y / k0;
    mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64-5*eccSquared*eccSquared*eccSquared/256));

    phi1Rad = mu	+ (3*e1/2-27*e1*e1*e1/32)*sin(2*mu)
                + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
                +(151*e1*e1*e1/96)*sin(6*mu);
    phi1 = phi1Rad * 180. / M_PI;

    N1 = a/sqrt(1-eccSquared*sin(phi1Rad)*sin(phi1Rad));
    T1 = tan(phi1Rad)*tan(phi1Rad);
    C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
    R1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
    D = x/(N1*k0);

    Lat = phi1Rad - (N1*tan(phi1Rad)/R1)*(D*D/2-(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
                    +(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720);
    Lat = Lat * 180. / M_PI;

    Long = (D-(1+2*T1+C1)*D*D*D/6+(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
                    *D*D*D*D*D/120)/cos(phi1Rad);
    Long = LongOrigin + Long * 180. / M_PI;

}


}
