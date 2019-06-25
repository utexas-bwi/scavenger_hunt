#ifndef SCAVENGER_HUNT_CLIENT_H
#define SCAVENGER_HUNT_CLIENT_H

#include <curl/curl.h>
#include <scavenger_hunt/rapidxml.hpp>
#include <scavenger_hunt/scavenger_hunt_structure.h>
#include <vector>

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

    @param hunt_name name of hunt you wish to participate in
    @param tasks list to populate with tasks
  */
  void get_hunt(std::string hunt_name, std::vector<Task> &tasks);

  /**
    Downloads validation feedback for a particular task.

    @param task task to retrieve feedback on
    @param proofs list to populate with feedback
  */
  void get_proofs(Task &task, std::vector<Proof> &proofs);

  /**
    Downloads the file that was submitted for a particular proof.

    @param proof proof to retrieve material for
    @param filepath folder to save file in
  */
  void download_proof_material(Proof &proof, std::string filepath);

  /**
    Uploads an image proof to Scavenger Hunt.

    @param file_path path to proof file (either an image or a video)
    @param task task you are proving completion of
    @param time time taken to complete task
    @return if a response was received from the server; not necessarily a
            successful upload
  */
  bool send_proof(std::string file_path, Task &task, double time);
};

#endif
