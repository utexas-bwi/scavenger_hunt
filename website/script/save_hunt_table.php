<?php
function typecheck($dbh, $task_type, $param_val) {
    // grab parameter name given task type  
    $query = "SELECT param_name from task_table where task_type='" . $task_type . "'";
    $stmt = $dbh->query($query);
    $stmt->setFetchMode(PDO::FETCH_ASSOC);
    $params = explode(", ", ($stmt->fetch())['param_name']);
    $values = [];
    // get possible values of parameter
    for ($pdx = 0; $pdx < count($params); ++$pdx) {
        $query = "SELECT possible_values from task_param_table where param_name='" . $params[$pdx] . "'";
        $stmt = $dbh->query($query);
        $stmt->setFetchMode(PDO::FETCH_ASSOC);
        $values = explode(", ", ($stmt->fetch())['possible_values']);
    }
    // check if value matches any of the possible values
    for ($idx = 0; $idx < count($values); ++$idx) {
        if ($param_val == $values[$idx]) {
            return true;
        }   
    }
    return false;
}

if ($_POST['save_hunt']) {
    $newrows = $_POST['hunt_table'];
    // Create connection
    include_once 'connect.php';
    $dbh = connect();
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    $results["error"] = false;
    try {
        if ($_POST['hunt_id'] >= 0) {
            // typecheck to ensure all parameters match task_type allowances
            for ($ndx = 0; $ndx < count($newrows); ++$ndx) {
                // figure out where the task type and parameters are in the array
                $task = 1;
                if (count($newrows[$ndx]) == 2) {
                    $task = 0;
                }   
                if (!typecheck($dbh, $newrows[$ndx][$task], $newrows[$ndx][$task + 1])) {
                    // do not save, return error message
                    $results["error"] = true;
                    $results["data"] = 'Failed to save hunt, parameter ' . $newrows[$ndx][1] . ' for task ' . $newrows[$ndx][0] . ' do not match allowed values.';
                    header("Content-type: application/json");
                    die(json_encode($results));
                }
            }
            // update name of hunt
            $sql = "UPDATE hunt_table SET hunt_name=? WHERE hunt_id=?";
            $name_array = [$_POST['hunt_name'], $_POST['hunt_id']];
            $dbh->prepare($sql)->execute($name_array);
            // get copy of old database
            $query = "SELECT * FROM hunt_instructions_table where hunt_id=" . $_POST['hunt_id']; 
            $stmt = $dbh->query($query);
            $stmt->setFetchMode(PDO::FETCH_ASSOC);
            $oldrows = [];
            while($oldrow = $stmt->fetch()) {
                array_push($oldrows, $oldrow);
            }
            // update hunt instruction rows
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
            $new_hunt = array(0, $_POST['hunt_name'], date('y-m-d'), (int)$_POST['user_id']);
            $dbh->prepare($sql)->execute($new_hunt);
            $query = "SELECT hunt_id FROM hunt_table where hunt_name='" . $_POST['hunt_name'] ."'";
            $stmt = $dbh->query($query);
            $stmt->setFetchMode(PDO::FETCH_ASSOC);
            $hunt_id = ($stmt->fetch())['hunt_id'];
            // insert all instructions into table
            echo "add ". count($newrows). " rows\n";
            for ($rdx = 0; $rdx < count($newrows); ++$rdx) {
                array_unshift($newrows[$rdx], 0);
                array_unshift($newrows[$rdx], $hunt_id);
                $sql = "INSERT into hunt_instructions_table VALUES (?, ?, ?, ?)";
                $dbh->prepare($sql)->execute($newrows[$rdx]);
            }
        }
    } catch (PDOException $e) {
        $results = array(
            "error" => true,
            "data" => $e->getMessage()
        );
        header("Content-type: application/json");
        die(json_encode($results));
    }
    // Close connection
    $dbh = null;
}
?>
