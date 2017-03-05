/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */

#ifndef _WGS_CONVERSION_H_
#define _WGS_CONVERSION_H_

namespace mod_utils {

// utility functions to convert geodetic to ECEF positions
void LLH2ECEF(double lat, double lon, double alt, double geo, double& x, double& y, double& z);

// utility functions to convert geodetic to UTM positions
void LL2UTM(double lat, double lon, double& x, double& y);

void ConvertECEFToLTP(double nlecef[3], double * nllat, double * nllon, double * nlalt );

void UTMtoLL(const double UTMNorthing, const double UTMEasting, const char* UTMZone, double& Lat,  double& Long );

}

#endif
