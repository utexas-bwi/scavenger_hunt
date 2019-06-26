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
    Gets the status of a single proof.

    @param id proof ID
    @return either PROOF_CORRECT, PROOF_INCORRECT, or PROOF_NOT_VALIDATED
  */
  proof_status_t get_proof_status(proof_id_t id);

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
    @return UID of submitted proof, or UPLOAD_FAILED if something went wrong
  */
  proof_id_t send_proof(std::string file_path, Task &task, double time);
};

#endif
