<?php
  for($x = 1; $x <= $numRows; $x++){
  
  //color of row
  if($x % 2 == 0){
    echo '<div class="leaderboard-row flex row-alt align-center">';
  } else {
    echo '<div class="leaderboard-row flex align-center">';
  }
  
  //position
  echo '<div class="row-position">';
  echo $x;
  echo '</div>';
  echo '<div class="row-collapse flex align-center">';

  //university name
  $university = $stmt -> fetch();
  $uni = $university['university'];
  echo '<div class="row-caller flex">';
  if($uni == "University of Texas at Austin")
    echo '<img class="avatar" src="../../public_html/images/ut-logo.png" />';
  else if($uni == "University of Pennsylvania")
    echo '<img class="avatar" src="../../public_html/images/upenn-logo.png" />';
  else if($uni == "University of Michigan")
    echo '<img class="avatar" src="../../public_html/images/mich-logo.png" />';
  else if($uni == "University of Southern California")
    echo '<img class="avatar" src="../../public_html/images/usc-logo.png" />';
  else if($uni == "Brown University")
    echo '<img class="avatar" src="../../public_html/images/brown-logo.png" />'; 
  echo '<div class="row-user">';
  echo $university['university'];
  echo '</div> </div>';

  // number of tasks
  echo '<div class="row-team">';
  echo $university['num_verified'];
  echo '</div>';

  // percent success
  echo '<div class="row-rank">';
  $percent = 0;
  if ($university['num_verified'] != 0)
    $percent = $university ['num_correct'] / $university['num_verified'] * 100;
  echo round($percent, 2) . "%";
  echo '</div> </div>';

  // total score
  echo '<div class="row-calls">';
  echo $university['score'];
  echo '</div> </div>';
  
  }
  echo '</div>'; 
?>