
/* functions to add to task table */

function addRow(cell) {
    const table = cell.parentNode.parentNode.parentNode;
    const row = document.getElementById("tr-hide").cloneNode(true);
    row.classList.remove("hide");
    row.classList.remove("table-line");
    table.appendChild(row);
}

function removeRow(cell) {
    const table = cell.parentNode.parentNode.parentNode;
    table.deleteRow(cell.parentNode.parentNode.rowIndex);
}

function saveChanges() {
    const table = document.getElementById("task-table");
    // TODO: export data to SQL TaskType database 
}

/* functions to add to hunt table */

function addTask(task) {
    const table = document.getElementById("hunt-table");
    const row = document.getElementById("tr-hide").cloneNode(true);
    row.childNodes[1].textContent = task;
    row.classList.remove("hide");
    row.classList.remove("table-line");
    table.appendChild(row);
}
