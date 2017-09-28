# Lab journal

## Markdown

Markdown is a formatting language for plain text where the syntax is also plain text. It is designed to be easily converted to HTML.

Github supports additional markdown syntax and details are listed here: [markdown cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)

A # denotes a header, multiple hashes (##) make smaller and smaller headers (H1, H2 etc).

Two underscores or asterisks on either side of the word will make it __bold__ and An underscore or asterisk on either side will make _italics_. (Really these are just defined as 'emphasis' in the markdown spec rather than bold or italics).

The syntax can be combined to make bold and italics text.

* A single asterisk at the start of the line followed by makes an unordered list item

+ A plus...

- or a minus also denote an unordered list

1. An ordered list is denoted by the number 1 followed by a fullstop. (1.)

A horizontal rule can be drawn by either 3 dashes, 3 asterisks or 3 underscores:

---

***

___


Link breaks are just created by newline characters (hitting enter once will insert 1 newline)

Links are denoted by the display text in square brackets followed by the url in normal brackets without a space. [This is a link](www.google.com)

The syntax for images is similar to links but with an exclamation mark at the start. The text in the square brackets is the alt text.
![alt text](www.example.com/image.jpg)

You can also define the image as a reference where you define a variable for the image url (in square brackets followed by a colon and then the url. ie: [logo]: image.png) and use this in the brackets.
![alt text][logo]
[logo]: www.example.com/image.jpg

> Block quotes are denoted by a > symbol.
> This is an example

`inline code is represented by back-ticks around the text`

HTML can also be used within markdown.

## Command line
Played around with commands.
`cd, ls, mkdir` etc
`ls -al` will show hidden files (files with a . at the front)

## Git
Git is a version control system.
`git init` created a git repository in the current directory
`git add` marks files to be committed
`git commit` commits file changes to the local repo
`git commit -m <message>` allows you to specify a message for the commit
`git add <name> <url of remote>` is used to add a remote (usually named origin)
`git push <remote name>` is used to push changes to a remote git server
`git push -u origin master` push the local master branch to the remote called origin with the 'upstream' flag set


## Hacking into the robot
Googled details about the Aldebaran nao and discovered the default hostname of nao.local and the default username and password which are 'nao'.

The robot it called chapman so worked out the hostname is chapman.local.

ssh'ed in with 'ssh nao@chapman.local' and typed in the password.

Ran the commands using the python command line interface (pressing enter after each line):

```
from naoqi import ALProxy
tts = ALProxy("ALTextToSpeech", "localhost", 9559)
tts.say("I've hacked you, robot!")
```
