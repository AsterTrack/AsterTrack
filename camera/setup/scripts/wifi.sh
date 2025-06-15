#!/bin/sh
# (c) Robert Shingledecker 2011-2012 v1.4
# Mods for open access storage and auto connect by Randy McEuen
# Mods for wait for connection and display hostname on router by Gerrelt
# Mods for configurable wpa_supplicant driver by Bela Markus
# Mods for connecting to best network by Seneral
. /etc/init.d/tc-functions

alias awk="busybox awk"
alias grep="busybox grep"

help() {
	echo "Usage:"
	echo "  Default select AP from menu and request IP via DHCP."
	echo "  -a Auto connect to first wifi.db entry via DHCP."
	echo "  -b Auto connect to wifi.db entry with the best signal quality via DHCP."
	echo "  -p Select AP from menu and prompt for IP configuration type."
	echo "  -w Wait indefinitely until lease is obtained"
	echo "  -? Displays this help message."
	exit 0
}

cleanup() {
	ifconfig "$WIFI" down 2>/dev/null
	for k in `ps | awk '/'${WIFI}'/{print $1}'`; do kill $k 2>/dev/null; done
}

checkroot
read TCUSER < /etc/sysconfig/tcuser
DB=/home/"$TCUSER"/wifi.db
PTMP=/tmp/wpa.$$
WPADRV=$(cat /etc/sysconfig/wifi-wpadrv)
export WPADRV

while getopts abpw? OPTION
do

	case ${OPTION} in
		a) MODE=auto ;;
		b) MODE=best ;;
		p) MODE=prompt ;;
		w) OBTAINLEASE=wait ;;
		*) help ;;
	esac
done
[ -n "$MODE" ] || MODE=menu
[ -n "$OBTAINLEASE" ] || OBTAINLEASE=try

CURRENTHOSTNAME=$(hostname -s)
[ -n "$CURRENTHOSTNAME" ] || CURRENTHOSTNAME=Tinycore

unset WIFI && CNT=0
until [ -n "$WIFI" ]
do
	[ $((CNT++)) -gt 10 ] && break || sleep 1
	WIFI="$(iwconfig 2>/dev/null | awk '{if (NR==1)print $1}')"
done
if [ -z "$WIFI" ]; then
	echo "No wifi devices found!"
	exit 1
fi

echo "Found wifi device $WIFI"
if [ ${MODE} == "menu" ]; then
	# Check if IP are already set (Access point may be connected or not)
	ifconfig|awk 'BEGIN{RS=""}/'${WIFI}'/'|grep -q 'inet addr'
	if [ "$?" == 0 ]; then 
		# Check if Acces point Associated!
		iwconfig ${WIFI}| grep -q  Not-Associated 
		if [ "$?" != 0 ]; then 
			echo
			echo "Already connected!"
			ifconfig ${WIFI}
			echo -n "Disconnect and Scan? (y/n): ";read ans
			[ "${ans:0:1}" != "y" ] && exit
			cleanup
			sleep 5
		fi
	fi
fi

echo "Standby for scan of available networks..."
ifconfig "$WIFI" up 2>/dev/null
(for i in `seq 5`
do
	iwlist "$WIFI" scanning
	[ "$?" == 0 ] && break
	sleep 1
done ) | awk -v wifi="$WIFI" -v dbfile="$DB" -v mode="$MODE" -v obtainlease="$OBTAINLEASE" -v currenthostname=$CURRENTHOSTNAME -v ptmp="$PTMP" '
BEGIN {
	RS="\n"
	FS=":"
	i = 0
	title = "Select Wifi Network"
	offset = 1
	if (mode == "auto" ) {
		if (getline dbitem < dbfile ) {
			split(dbitem,field,"\t")         
			autoconnect = field[1]
			mypass = field[2]
			close(dbfile)
		}
	}
	else if (mode == "best" ) {
		m = 1
		while (getline dbitem < dbfile ) {
			split(dbitem,field,"\t")
			optid[m] = field[1]
			optpw[m] = field[2]
			m++
		}
		close(dbfile)
	}
}
function read_console() {
	"head -1 < /dev/tty" | getline results
	close("head -1 < /dev/tty")
	return results
}

function associate(t,d,s,p,c) {
	print p > ptmp
	keyphrase=" key restricted "
	if ( p == "" ) { keyphrase = "" }
	selchannel=" channel "
	if ( c == "" ) { selchannel = "" }
	if (t == "WPA") {
		system("wpa_passphrase " s " < " ptmp " > /etc/wpa_supplicant.conf")
		system("wpa_supplicant -i " d " -c /etc/wpa_supplicant.conf -B  -D $WPADRV >/dev/null 2>&1")
	} else {
		system("iwconfig " d " essid " s keyphrase p selchannel c)
	}
	for (try=1; try<20; try++) {
		printf(".")
		results = system("iwconfig " d "| grep -q Not-Associated ")
		if ( results == 1 ) { try = 20 }
		system("sleep 2")
	}
	printf("\n")
}

function rsort(qual,sid,enc,chan,type,n,	i,j,t) {
	print("Sorting")
	for (i = 2; i <= n; i++)
		for (j = i; j > 1 && qual[j]+0 > qual[j-1]+0; j--) {
			# swap qual[j] and qual[j-1]
			t = qual[j]; qual[j] = qual[j-1]; qual[j-1] =t
			t = sid[j]; sid[j] = sid[j-1]; sid[j-1] =t
			t = enc[j]; enc[j] = enc[j-1]; enc[j-1] =t
			t = chan[j]; chan[j] = chan[j-1]; chan[j-1] =t
			t = type[j]; type[j] = type[j-1]; type[j-1] =t
		}
}

function setipaddresses(d) {
	do {
		addr ="192.168.1.1"
		printf("Enter host ip address (i.e " addr ") : ")
		ipaddr = read_console()
		if (ipaddr == "") { ipaddr = addr } 
		mask ="255.255.255.0"
		printf("Enter host net mask (i.e " mask ") : ")
		netmask = read_console()
		if (netmask == "") { netmask = mask } 
	}
	while (system("ifconfig " d  " " ipaddr " netmask " netmask " up") != 0)
	do {
		addr ="192.168.1.254"
		printf("Enter ip gateway address (i.e " addr ") : ")
		ipaddr = read_console()
		if (ipaddr == "") { ipaddr = addr } 
	}
	while (system("route add default gw " ipaddr ) != 0)
	
	#Fortunaly nslookup belongs to busybox so no need to check if it is presents
	do {
		addr = "127.0.0.1"
		printf("Enter DNS ip address (i.e " addr ") : ")
		ipaddr = read_console()
		if (ipaddr == "") { ipaddr = addr } 
	}
	while (system("nslookup " addr " " ipaddr " >/dev/null 2>&1 ") != 0)
}

# awk Main()
{
	if ($1 ~ /Cell/) 
		if ( i == 0  || sid[i] != "" ) i++
	if ($1 ~ /Frequency/) {
		split($2,c," ")
		chan[i] = c[4]
		gsub("\)","",chan[i])
	}
	if ($1 ~ /Quality/) {
		q = $2
		if (index($1,"=")) {
			split($1,c,"=")
			q = c[2]
		}
		split(q,c," ")
		qual[i] = c[1]
	}
	if ($1 ~ /Encr/){
		enc[i] = $2
	}
	if ($1 ~ /ESSID/) {
		sid[i] = $2
		gsub("\"","",sid[i])
	}
	if ($2 ~ /WPA/ ) type[i]="WPA"
}
END {
	if ( obtainlease == "try" ) {
	    lease = "-n "
		print("Set to try a few times to obtain a lease." )
	} else if ( obtainlease == "wait" ) {
	    lease = ""
		print("Set to try indefinitely until lease is obtained." )
	}
	if ( mode == "auto" ) {
		print("Attempting auto connection with " autoconnect)
		for (i in sid) {
			gsub("\134\134","\134\134",sid[i])	# Backslash (must come first)
			gsub(" ","\\ ",sid[i])			# Space
			gsub("\047","\\\047",sid[i])		# Single quote
			gsub("&","\\\\&",sid[i])		# Ampersand
			gsub("!","\\!",sid[i])			# Exclamation mark
			gsub("?","\\?",sid[i])			# Question mark
			gsub("\\$","\\$",sid[i])		# Dollar sign
			gsub("\\*","\\*",sid[i])		# Asterisk
			gsub("\\=","\\=",sid[i])		# Equal sign
			if ( autoconnect == sid[i] ) {
				associate(type[i],wifi,sid[i],mypass,chan[i])
				system( "udhcpc " lease "-i " wifi " -x hostname:" currenthostname " 2>/dev/null" )
				exit
			}
		}
		print("Could not connect to desired network!")
		exit
	} else if ( mode == "best" ) {
		rsort(qual,sid,enc,chan,type, NR)
		for (j=1; j<15; j++) {
			if ( j <= i ) {
				gsub("\134\134","\134\134",sid[j])	# Backslash (must come first)
				gsub(" ","\\ ",sid[j])			# Space
				gsub("\047","\\\047",sid[j])		# Single quote
				gsub("&","\\\\&",sid[j])		# Ampersand
				gsub("!","\\!",sid[j])			# Exclamation mark
				gsub("?","\\?",sid[j])			# Question mark
				gsub("\\$","\\$",sid[j])		# Dollar sign
				gsub("\\*","\\*",sid[j])		# Asterisk
				gsub("\\=","\\=",sid[j])		# Equal sign
				for (k in optid) {
					if ( optid[k] == sid[j] ) {
						print("Found network " sid[j] " with strength " qual[j] " to connect to.")
						associate(type[j],wifi,sid[j],optpw[k],chan[j])
						system( "udhcpc " lease "-i " wifi " -x hostname:" currenthostname " 2>/dev/null" )
						exit
					}
				}
			}
		}
		print("Could not find any known network!")
		for (j=1; j<15; j++) {
			if ( j <= i ) {
				print("  Found network " sid[j] " with strength " qual[j] ".")
			}
		}
		exit
	} else
	do {
		if ( NR < 1 ) {
			selection = "q"
			break
		}
		rsort(qual,sid,enc,chan,type, NR)
		system ("busybox clear")
		printf "\n%s\n\n", title
		printf "    %-20s\t%5s\t%s\t%s\t%s\t%s\n", "ESSID", "Enc", "Qual", "Channel Type"
		for (l=1; l<15; l++) {
			++j
			if ( j <= i ) printf "%2d. %-20s\t %4s\t %2d\t %2d\t%s\n", j, sid[j], enc[j], qual[j], chan[j], type[j]
		}
		printf "\nEnter selection ( 1 - %s ) or (q)uit", i
		if ( i > 15 ) printf ", (n)ext, (p)revious: "
		printf ": "
		selection = read_console()
		if (selection == "q") break
		if (selection == "p") {
			if ( j > 15 )
				j = j - 30
			else
				j = 0
			continue
		}
		if (selection == "n" || selection == "") {
			if ( j > NR )
				j = j - 15
			continue
		}
		selection = selection + 0
		if (selection+0 < 1 || selection+0 > i ) j = j - 15
	} while (selection < 1 || selection > i)
	if ( offset == 1 && selection != "q") {
		sid_display = sid[selection]
		password = ""
		newitem=""
		gsub("\134\134","\134\134",sid[selection])	# Backslash (must come first)
		gsub(" ","\\ ",sid[selection])			# Space
		gsub("\047","\\\047",sid[selection])		# Single quote
		gsub("&","\\\\&",sid[selection])		# Ampersand
		gsub("!","\\!",sid[selection])			# Exclamation mark
		gsub("?","\\?",sid[selection])			# Question mark
		gsub("\\$","\\$",sid[selection])		# Dollar sign
		gsub("\\*","\\*",sid[selection])		# Asterisk
		gsub("\\=","\\=",sid[selection])		# Equal sign
		if ( enc[selection] == "on" ) {
			while ( getline dbitem < dbfile > 0 ) {
				split(dbitem,field,"\t")
				if (sid[selection] == field[1] ) {
					password = field[2]
					break
				}
			}
			close(dbfile)
			if ( password == "" ) {
				while (length(password) < 8 || length(password) > 63) {
					printf("Enter password for " sid_display" (8 to 63 characters): ")
					password = read_console()
					newitem=1
				}
			}
			printf("Sending credentials to requested access point %s", sid_display)
			
			associate(type[selection],wifi,sid[selection],password,chan[selection])
		} else {
			newitem=1
			while ( getline dbitem < dbfile > 0 ) {
				split(dbitem,field,"\t")
				if (sid[selection] == field[1] ) {
					newitem=""
					break
				}
			}
			close(dbfile)
			associate(type[selection],wifi,sid[selection],password,chan[selection])
		}
		if ( mode == "prompt" ) {
			#Choose automatic or manual setting IP adresses
			printf "(a)utomatic via DHCP or (m)anual IP adresses configuration ? "
			ans = read_console()
			if (ans == "m") {
				setipaddresses(wifi)
			}
			else {
				results = system( "udhcpc " lease "-i " wifi " -x hostname:" currenthostname " 2>/dev/null")
				if ( results == 0 )
					if ( newitem == 1 ) 
						printf("%s\t%s\t%s\n", sid[selection] ,password, type[selection] ) >> dbfile
			}
		}
		else {
			results = system( "udhcpc " lease "-i " wifi " -x hostname:" currenthostname " 2>/dev/null")
			if ( results == 0 )
				if ( newitem == 1 ) 
					printf("%s\t%s\t%s\n", sid[selection] ,password, type[selection] ) >> dbfile
		
		}
	}
} '
ifconfig|awk 'BEGIN{RS=""}/'${WIFI}'/'|grep -q 'inet addr'
if [ "$?" == 1 ]; then 
	if [ ${MODE} == "menu" ]; then
		echo "Failed to connect."
		sleep 3
	fi
	cleanup
	exit 1
fi	
