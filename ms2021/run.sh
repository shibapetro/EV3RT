#!/usr/bin/env bash
trap 'killbackground > /dev/null 2>&1' EXIT

timeout() {
    time=$1

    # start the command in a subshell to avoid problem with pipes
    # (spawn accepts one command)
    command="/bin/sh -c \"${@:2}\""

    expect -c "set timeout $time; spawn -noecho $command; expect timeout { exit 124 } eof; catch wait result; exit [lindex \$result 3]"
}

killbackground(){
    local jobs=`jobs -p`
    for job in $jobs; do
        killpstree $job
    done
}

killpstree(){
    local children=`pgrep -P $1`
    for child in $children; do
        killpstree $child
    done
    kill $1
}

usage_exit() {
        echo "Usage: $0 [-r] [-c count] [-t maxtime] [-s speed] [-p p_value] [-i i_value] [-d d_value] [-j dest] [-f]" 1>&2
        echo "  -r rightコース走行時に指定" 1>&2
        echo "  count 繰り返し回数" 1>&2
        echo "  maxtime make開始から走行打ち切りまでの実時間" 1>&2
        echo "  speed 走行速度" 1>&2
        echo "  p_valud ライントレースPID制御用のP定数" 1>&2
        echo "  i_valud ライントレースPID制御用のI定数" 1>&2
        echo "  d_valud ライントレースPID制御用のD定数" 1>&2
        echo "  dest 走行体ジャンプ先（0: normal, 1: slalom, 2: garage）" 1>&2
        echo "  -f フーリエ解析用ログ出力（10ms毎）時に指定" 1>&2
        echo "" 1>&2
        echo "  ログは${DSTDIR}に格納されています" 1>&2
        echo "  ${MAKELOG}_(順序番号).${EXT}は、make時のログです" 1>&2
        echo "  ${BTLOG}_(順序番号).${EXT}は、走行体のログです" 1>&2
        echo "  graph.shでログをグラフ化できます" 1>&2
        echo "  グラフ化にはgnuplotが必要なので、未導入の場合は導入して下さい" 1>&2
        exit 1
}

if [ -z "$ETROBO_ENV" ]; then
    echo "etrobo environment is not available."
    exit 1
fi

SRCDIR=${ETROBO_HRP3_WORKSPACE}/ms2021
DSTDIR=${ETROBO_HRP3_WORKSPACE}/ms2021/work
MAKELOG="makelog"
BTLOG="btlog"
COND="cond"
EXT="txt"
SEQ=1

# default values for arguments able to set by command line
LR=""
COUNT=1
MAXTIME=65
SPEED=55
P=0.75
I=0.39
D=0.08
JUMP=0
FOURIER=""

cd $ETROBO_ROOT
if [ ! -d $DSTDIR ]; then
    mkdir -p $DSTDIR
fi

BASE=${MAKELOG}_${SEQ}

while getopts rc:t:s:p:i:d:j:fh OPT
do
    case $OPT in
        r)  LR="right"
            ;;
        c)  COUNT=$OPTARG
            ;;
        t)  MAXTIME=$OPTARG
            ;;
        s)  SPEED=$OPTARG
            ;;
        p)  P=$OPTARG
            ;;
        i)  I=$OPTARG
            ;;
        d)  D=$OPTARG
            ;;
        j)  JUMP=$OPTARG
            ;;
        f)  FOURIER="-DFOURIER"
            ;;
        h)  usage_exit
            ;;
        \?) usage_exit
            ;;
    esac
done

shift $((OPTIND - 1))

# check the validitity of argument
for ARG in $COUNT $MAXTIME $SPEED; do
    if echo "$ARG" | grep -q "^[0-9]\+$"; then
        continue
    else
        echo "$ARG is not a positive integer"
        exit 1
    fi
done
for ARG in $P $I $D; do
    if echo "$ARG" | grep -q "^[0-9]\+\.\?[0-9]*$"; then
        continue
    else
        echo "$ARG is not a positive number"
        exit 1
    fi
done
for ARG in $JUMP; do
    if echo "$ARG" | grep -q "^[0-2]$"; then
        continue
    else
        echo "$ARG is out of range"
        exit 1
    fi
done

sim
curl -X POST -H "Content-Type: application/json" -d @${SRCDIR}/init_JUMP_${JUMP}.json http://localhost:54000

for N in `seq ${COUNT}`; do
    while ls $DSTDIR | grep -w $BASE >/dev/null; do
    SEQ=`expr $SEQ + 1`
    BASE=${MAKELOG}_${SEQ}
    done

    echo P=${P} I=${I} D=${D} Speed=${SPEED} > ${DSTDIR}/${COND}_${SEQ}.${EXT}
    export USER_COPTS="-DP_CONST=${P}D -DI_CONST=${I}D -DD_CONST=${D}D -DSPEED_NORM=${SPEED} -DJUMP=${JUMP} ${FOURIER}"
    btcat $LR > ${DSTDIR}/${BTLOG}_${SEQ}.${EXT} &
    timeout $MAXTIME make $LR app=ms2021 sim up 2>&1 | tee ${DSTDIR}/${MAKELOG}_${SEQ}.${EXT}
done