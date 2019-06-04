#include "scavenger_hunt/scavenger_hunt.h"
#include <iostream>

int main(int argc, char** argv) {
    Task t("Find Object",
           "Locate an object by name.",
           "Image",
           "Submit an image of the object with a box drawn around it.",
           100,
           101);
    t.add_parameter("target object", "soda can");
    std::cout << t;

    ScavengerHunt hunt("Longhorn Hunt");
    hunt.add_task(t);
    std::cout << hunt.get_name() << " has " << hunt.size() << " tasks" << std::endl;
    std::cout << hunt[0];

    ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
    client.send_proof("bottle.png", hunt[0]);
}
