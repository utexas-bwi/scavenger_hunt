<?php

$xw = new XMLWriter();
$xw->openMemory();
$xw->startDocument("1.0");
$xw->startElement('book');
$xw->text("example");
$xw->endElement();
$xw->endDocument();
echo $xw->outputMemory();

?>