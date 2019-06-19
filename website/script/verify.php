<?php
  include 'connect.php';
  $dbh = connect();

  $image_extensions = array("png", "jpg", "jpeg");
  $video_extensions = array("mp4");

  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  // obtains the files that have not yet been verified by a user
  $query = "SELECT * FROM proof_table WHERE verified = 0";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);

  $proofs = 0;

  // goes through the filenames of proofs that have not been verified and displays on page
  while($item = $stmt -> fetch()){
    $imageFilename = $item['filename'];
    $proofId = $item['proof_id'];
    $huntInstrId = $item['hunt_instr_id'];
    $uploaderId = $item['uploader_id'];
    $extension = strtolower(pathinfo($imageFilename, PATHINFO_EXTENSION));
    if (in_array($extension, $image_extensions))
      include 'components/image-verify.html';
    else
      include 'components/video-verify.html';
    $proofs += 1;
  }

  if ($proofs == 0)
    include 'components/no-proofs.html';
?>
