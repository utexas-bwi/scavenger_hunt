<?php
  include_once 'connect.php';
  $dbh = connect();

  $imageExtensions = array("png", "jpg", "jpeg");
  $videoExtensions = array("mp4");

  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);


  // obtains the files that have not yet been verified by a user
  if ($_POST["user_specific"])
    $query = "SELECT * FROM proof_table WHERE (verified=0 and uploader_id = " . $_POST["user_id"] . ")";
  else
    $query = "SELECT * FROM proof_table WHERE (verified=0 and uploader_id != " . $_POST["user_id"] . ")";

  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);

  $proofs = 0;

  echo '<table style="width:100%" class="table">';
  echo '<tr>';
  echo '<th>Proof</th>';
  echo '<th>Task Name</th>';
  echo '<th>Description</th>';
  echo '<th>Parameters</th>';
  echo '<th>Status</th>';
  echo '</tr>';

  // goes through the filenames of proofs that have not been verified and displays on page
  while($item = $stmt -> fetch()){
    $proofId = $item['proof_id'];
    $proofFilename = $item['filename'];
    $uploaderId = $item['uploader_id'];
    $huntInstrId = $item['hunt_instr_id'];

    $taskQuery = $dbh->query("SELECT * FROM hunt_instructions_table WHERE hunt_instr_id=" . $huntInstrId);
    $taskQuery->setFetchMode(PDO::FETCH_ASSOC);
    $task = $taskQuery->fetch();
    $taskParameter = $task['param_value'];
    $taskName = $task['task_type'];
    $descriptionQuery = $dbh->query("SELECT * from task_table WHERE task_type='" . $taskName . "'");
    $descriptionQuery->setFetchMode(PDO::FETCH_ASSOC);
    $taskType = $descriptionQuery->fetch();
    $taskDescription = $taskType['description'];
    $extension = strtolower(pathinfo($proofFilename, PATHINFO_EXTENSION));

    if (in_array($extension, $imageExtensions))
      include '../public_html/components/verify-proof-table-row-image.html';
    else
      include '../public_html/components/verify-proof-table-row-video.html';

    // $imageFilename = $item['filename'];
    // $proofId = $item['proof_id'];
    // $huntInstrId = $item['hunt_instr_id'];
    // $uploaderId = $item['uploader_id'];
    // $extension = strtolower(pathinfo($imageFilename, PATHINFO_EXTENSION));
    // if (in_array($extension, $image_extensions))
    //   include '../public_html/components/image-verify.html';
    // else
    //   include '../public_html/components/video-verify.html';
    $proofs += 1;
  }

  echo '</table>';

  include '../public_html/components/verify-proof-table-submit.html';
?>
