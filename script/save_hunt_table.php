<?php
if ($_POST['save_hunt']) {
    $newrows = $_POST['hunt_table'];
    // Create connection
    include 'connect.php';
    include 'auth.php';
    $dbh = connect();
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    try {
        if ($_POST['hunt_id'] >= 0) {
            // get copy of old database
            $query = "SELECT * FROM hunt_instructions_table where hunt_id=" . $_POST['hunt_id']; 
            $stmt = $dbh->query($query);
            $stmt->setFetchMode(PDO::FETCH_ASSOC);
            $oldrows = [];
            while($oldrow = $stmt->fetch()) {
                array_push($oldrows, $oldrow);
            }
            // update rows
            for ($odx = 0; $odx < count($oldrows); ++$odx) {
                for ($rdx = 0; $rdx < count($newrows); ++$rdx) {
                    if ($newrows[$rdx][0] == $oldrows[$odx]['hunt_instr_id']) {
                        echo $rdx. " has same id as " . $odx . "\n";
                        // fix ordering of values
                        $id = $newrows[$rdx][0];
                        array_splice($newrows[$rdx], 0, 1);
                        array_push($newrows[$rdx], $id);
                        // update row
                        $sql = "UPDATE hunt_instructions_table SET task_type=?, param_value=? where hunt_instr_id=?";
                        $dbh->prepare($sql)->execute($newrows[$rdx]);
                        // remove rows we changed
                        array_splice($oldrows, $odx, 1);
                        array_splice($newrows, $rdx, 1);
                        $odx = -1;
                        $rdx = -1;
                    }
                }
            }
            // add/remove any leftover rows
            if (!empty($oldrows)) {
                // remove all oldrows
                for ($odx = 0; $odx < count($oldrows); ++$odx) {
                    echo "removing " . $oldrows[$odx]['id'] . "\n";
                    $sql = "DELETE from hunt_instructions_table WHERE hunt_instr_id=" . $oldrows[$odx]['hunt_instr_id'];
                    $dbh->prepare($sql)->execute();
                }
            }
            if (!empty($newrows)) {
                // add all newrows
                echo "add ". count($newrows). " rows\n";
                for ($rdx = 0; $rdx < count($newrows); ++$rdx) {
                    array_unshift($newrows[$rdx], 0);
                    array_unshift($newrows[$rdx], $_POST['hunt_id']);
                    $sql = "INSERT into hunt_instructions_table VALUES (?, ?, ?, ?)";
                    $dbh->prepare($sql)->execute($newrows[$rdx]);
                }
            }
        } else {
            // create new hunt
            $sql = "INSERT into hunt_table VALUES (?, ?, ?, ?)";
            $new_hunt = array(0, $_POST['hunt_name'], date('y-m-d'), getUserId());
            $dbh->prepare($sql)->execute($new_hunt);
            $query = "SELECT hunt_id FROM hunt_table where hunt_name='" . $_POST['hunt_name'] ."'";
            $stmt = $dbh->query($query);
            $stmt->setFetchMode(PDO::FETCH_ASSOC);
            $hunt_id = $stmt->fetch()['hunt_id'];
            // insert all instructions into table
            echo "add ". count($newrows). " rows\n";
            for ($rdx = 0; $rdx < count($newrows); ++$rdx) {
                array_unshift($newrows[$rdx], 0);
                array_unshift($newrows[$rdx], $name);
                $sql = "INSERT into hunt_instructions_table VALUES (?, ?, ?, ?)";
                $dbh->prepare($sql)->execute($newrows[$rdx]);
            }
        }
    } catch (PDOException $e) {
        die($e->getMessage());
    }
    // Close connection
    $dbh = null;
}
?>
