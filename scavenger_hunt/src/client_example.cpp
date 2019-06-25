#include <iostream>
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
  client.send_proof("bottle.png", tasks[0], 60.0);

  // Retrieving feedback for proofs is equally as trivial. If the vector is
  // empty after this step, no proofs for the specified task have been validatd.
  std::vector<Proof> proofs;
  client.get_proofs(tasks[0], proofs);

  for (int i = 0; i < proofs.size(); i++) {
    Proof &proof = proofs[i];
    bool is_correct = proof.get_correct(); // Was I right?

    // The original file uploaded as proof can be retrieved if necessary
    client.download_proof_material(proof, "my_validated_proofs/");
    std::ofstream proof_file;
    proof_file.open("my_validated_proofs/" + proof.get_filename());
    proof_file.close();
  }
}
