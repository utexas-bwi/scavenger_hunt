# scavenger\_hunt

## Packages Needed

* php
* mysql-server
* php-curl
* php-xml
* apache2

## Directory Structure

* public\_html: contains all website files
    * css: contains stylesheets
    * images: contains all images    
* script: contains php and javascript scripts used on the webpage

## Notes on Ozil

On a Linux machine (such as Ozil), you will have to copy the files from this repository into the /var/www/html folder on your computer to have it displayed on localhost. This folder requires sudo access to copy the files.

The following packages have been downloaded on Ozil: php, apache2, mysql, php-curl, php-xml.

When configuring apache2, you will have to add the following line to /etc/apache2/apache2.conf:

`AddHandler application/x-httpd-php .htm .html`

The mysql password is the same as the sudo password for the bwilab account on Ozil.

## Viewing the Website on Kane/Ozil

Currently, the website is hosted on Ozil, Kane, and Kif, so you will have to have a BWI account to view the website. You can view the website by ssh-ing into whichever of the three machines you wish with the following command:

`ssh -L 8080:128.83.143.224:80 [username]@[ozil/kane/kif].csres.utexas.edu`

Then, open a web browser and go to the website address `http://localhost:8080`, and you will be able to see the website.
