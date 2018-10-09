/*
 * testMapping.C
 *
 *  Created on: Oct 9, 2018
 *      Author: zenker
 */

#include <iostream>
#include "ChimeraTK/MapFileParser.h"

using namespace std;

int main(){
  ChimeraTK::MapFileParser parser;
  auto map = parser.parse("sigma_phi_FI004250.map");
  for(auto it = map->begin(); it != map->end(); it++){
    cout << it->getRegisterName() << "\t start: " << it->address << "\t size: " << it->nElements <<  endl;
  }
}
