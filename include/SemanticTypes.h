#ifndef SEMANTIC_TYPES_HEADER
#define SEMANTIC_TYPES_HEADER

#include <set>
#include <string>
#include <map>

using namespace std;

std::map<int,std::string> valid_semantic_types{
    make_pair(85,"chair"),
    make_pair(455,"table"),
    make_pair(138,"desk"),
    make_pair(442,"stool"),
    make_pair(155,"door"),
    make_pair(391,"shelf"),
    make_pair(508,"wardrobe"),
    make_pair(423,"sofa"),
    make_pair(298,"object"),
    make_pair(326,"piano"),
    make_pair(452,"suitcase"),
    make_pair(15,"bag"),
    make_pair(110,"commode"),
    make_pair(47,"blanket"),
    make_pair(59,"box"),
    make_pair(369,"roll"),
    make_pair(456,"table lamp"),
    make_pair(103,"clutter"),
    make_pair(360,"radiator")
};


#endif  