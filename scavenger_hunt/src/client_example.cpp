#include <iostream>
#include <fstream>
#include <scavenger_hunt/scavenger_hunt.h>

int main(int argc, char** argv) {
  // Create a new client object and sign in with your website credentials.
  ScavengerHuntClient client("your@email.com", "password");

  // Create an empty vector of tasks for the client to populate with hunt data.
  std::vector<Task> tasks;

  // Scavenger hunts are downloaded by name. We've created a never-expiring hunt
  // called Bottle Hunt to test with. It contains a single task to find a bottle
  // and take a picture of it.
  client.get_hunt("Bottle Hunt", tasks);

  // Task objects contain all of the information necessary to complete them.
  for (int i = 0; i < tasks.size(); i++) {
    Task &task = tasks[i];

    // Insert a task into a stream to get a pretty-printed summary.
    std::cout << task << std::endl;

    // Access to task details. See
    // include/scavenger_hunt/scavenger_hunt_structure.h for the full docs.
    std::string task_name = task.get_name();
    std::string task_description = task.get_description();
    std::string target_object = task.get_parameter_value("object"); // Specific to the "Find Object" task
  }

  // Upload proof of each completed task with send_proof. This method takes a
  // path to a proof file (either an image or a video), the task in question,
  // and the time taken to complete the task in seconds.
  proof_id_t id = client.send_proof("bottle.jpeg", tasks[0], 60.0);

  // The upload process generates a unique ID for the proof that can be used to
  // get feedback on it.
  proof_status_t status = client.get_proof_status(id);

  if (status == PROOF_CORRECT)
    std::cout << ":)";
  else if (status == PROOF_INCORRECT)
    std::cout << ":(";
  else if (status == PROOF_NOT_VALIDATED)
    std::cout << ":|";

  // Alternatively, all of the proofs submitted for a particular task that have
  // been validated can be downloaded in bulk with get_proofs.
  std::vector<Proof> proofs;
  client.get_proofs(tasks[0], proofs);

  for (int i = 0; i < proofs.size(); i++) {
    Proof &proof = proofs[i];
    bool is_correct = proof.get_correct(); // Was I right?

    // The original file uploaded as proof can be retrieved if necessary.
    std::string download_directory = "/home/my_downloaded_proofs";
    client.download_proof_material(proof, download_directory);
    std::string path_to_proof = download_directory + "/" + proof.get_filename();
  }
}
