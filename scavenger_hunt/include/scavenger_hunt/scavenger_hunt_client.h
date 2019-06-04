#ifndef SCAVENGER_HUNT_CLIENT_H
#define SCAVENGER_HUNT_CLIENT_H

#include <curl/curl.h>
#include <scavenger_hunt/rapidxml.hpp>
#include <scavenger_hunt/scavenger_hunt_structure.h>

/**
  Your robot's connection to Scavenger Hunt.
*/
class ScavengerHuntClient {
public:
  /**
    Creates a new connection to the Scavenger Hunt service. We recommend only
    creating one connection per program.

    The account details used to log in to the service should be unique to your
    robot. This is the account its proofs will be submitted under, and points
    earned for correct proofs will be awarded to the university associated with
    the account.

    @param email account email
    @param password account password--don't worry, we're very careful with this
  */
  ScavengerHuntClient(std::string email, std::string password);

  /**
    Downloads a hunt from Scavenger Hunt.
    TODO: actually write this

    @param hunt_name name of hunt you wish to participate in
    @return hunt data
  */
  ScavengerHunt* get_hunt(std::string hunt_name);

  /**
    Uploads an image proof to Scavenger Hunt.

    @param image_path path of proof
    @param task task you are proving completion of
    @return if a response was received from the server; not necessarily a
            successful upload
  */
  bool send_proof(std::string image_path, Task &task);
};

#endif
