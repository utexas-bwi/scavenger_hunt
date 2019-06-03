#ifndef SCAVENGER_HUNT_H
#define SCAVENGER_HUNT_H

#include <curl/curl.h>
#include <json/json.h>

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
    @param password account password--don't worry, we're very secure with this
  */
  ScavengerHuntClient(std::string email, std::string password);

  /**
    @brief not working currently; do not call or computer explode
  */
  void get_hunts();

  /**
    Uploads an image proof to Scavenger Hunt.

    @param image_path path of proof
    @param task_id ID of task you are proving completion of
  */
  bool send_proof(std::string image_path, unsigned int task_id);
};

#endif
