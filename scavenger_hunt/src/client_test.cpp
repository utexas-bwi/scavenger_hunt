#include "scavenger_hunt/scavenger_hunt.h"
#include <iostream>

int main(int argc, char** argv) {
    ScavengerHuntClient client("aaaaa@aaaaaaa.com", "sick robots");
    client.send_proof("bottle.png", 100);
}
