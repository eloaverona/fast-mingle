//
// Created by Sen on 6/14/17.
//

#ifndef MINGLE_EDGEIDGENERATOR_H
#define MINGLE_EDGEIDGENERATOR_H


#include <printf.h>
#include <stdlib.h>

class EdgeIdGenerator {
private:
    static unsigned long lastId;

public:
    /**
   * Build new EdgeIdGenerator
   */
    EdgeIdGenerator();

    EdgeIdGenerator(long lastId);



    char *generateNewID(){
        unsigned long newIDnumber = lastId + 1;

        lastId = newIDnumber;
        char *id;
        id = (char *) malloc((sizeof(char) + sizeof(unsigned long)) + 2);
        snprintf(id, sizeof(char) + sizeof(unsigned long) + 2, "e%lu", newIDnumber);
        return id;
    }
};


#endif //MINGLE_EDGEIDGENERATOR_H
