
#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Core>

#include <iostream>
#include <catch2/catch.hpp>

#include "lidar.hpp"
#include "Eig.hpp"

SCENARIO("Lidar Scan on known map") {
    GIVEN("A map with known occupancy"){
        Map map;

        // Intialise map paramaters
        map.x.resize(400);
        map.y.resize(400);        
        map.dx = 0.5;
        map.dy = map.dx;
        map.x0 = 0;
        map.y0 = map.x0;
        map.x.setLinSpaced(400,map.x0,(400-1)*map.dx);
        map.y.setLinSpaced(400,map.y0,(400-1)*map.dy);
        // std::cout << map.x << std::endl;
        map.numX = 400;
        map.numY = map.numX;

        std::string path = "navigation/data/testMap.csv";
        map.z = readCSV(path,400,400); 

        Scanner M8;
        M8.x0 = 0.0;
        M8.y0 = 0.0;
        M8.psi0 = 0.0;
        M8.startDeg = 0.0;
        M8.resDeg = 1.0;
        M8.maxRange = 150.0;
        M8.numScans = 360;

        Pose pose;
        pose.x = 100;
        pose.y = 100;
        pose.psi = 90;

        Eigen::VectorXd range, xr, yr,C;

        findRange(map, M8, pose, range, xr, yr, C);

        CHECK(range(  0) == Approx(             12.5));
        CHECK(range(  1) == Approx( 5.50083780424149));
        CHECK(range(  2) == Approx( 5.50335249364352));
        CHECK(range(  3) == Approx( 5.50754790298857));
        CHECK(range(  4) == Approx( 5.51343043944645));
        CHECK(range(  5) == Approx( 5.52100910648841));
        CHECK(range(  6) == Approx( 8.04406623650813));
        CHECK(range(  7) == Approx( 8.06007860367079));
        CHECK(range(  8) == Approx( 15.6523273740386));
        CHECK(range(  9) == Approx( 6.39245322149965));
        CHECK(range( 10) == Approx( 6.09255967131447));
        CHECK(range( 11) == Approx( 6.11230016973128));
        CHECK(range( 12) == Approx( 6.13404356919018));
        CHECK(range( 13) == Approx( 6.15782464676035));
        CHECK(range( 14) == Approx( 6.18368177609939));
        CHECK(range( 15) == Approx(  12.940952255126));
        CHECK(range( 16) == Approx( 9.06988819635825));
        CHECK(range( 17) == Approx( 8.88837993014076));
        CHECK(range( 18) == Approx( 8.93742890602527));
        CHECK(range( 19) == Approx(  8.9897757900867));
        CHECK(range( 20) == Approx( 8.77141320048926));
        CHECK(range( 21) == Approx( 8.56915994909623));
        CHECK(range( 22) == Approx( 8.62827794142067));
        CHECK(range( 23) == Approx( 8.69088301924237));
        CHECK(range( 24) == Approx( 18.0614985953498));
        CHECK(range( 25) == Approx(  7.0986047494575));
        CHECK(range( 26) == Approx( 6.84351609811458));
        CHECK(range( 27) == Approx( 6.73395742580616));
        CHECK(range( 28) == Approx( 6.79542030413423));
        CHECK(range( 29) == Approx( 6.86012440723992));
        CHECK(range( 30) == Approx( 6.92820323027551));
        CHECK(range( 31) == Approx( 11.6663339721533));
        CHECK(range( 32) == Approx(  11.791784033621));
        CHECK(range( 33) == Approx( 11.9236329283595));
        CHECK(range( 34) == Approx( 34.9803205066133));
        CHECK(range( 35) == Approx( 9.58895737591605));
        CHECK(range( 36) == Approx( 9.35715889187244));
        CHECK(range( 37) == Approx( 9.39101743617169));
        CHECK(range( 38) == Approx( 5.68494235918961));
        CHECK(range( 39) == Approx( 5.56155505173012));
        CHECK(range( 40) == Approx( 5.44503339401144));
        CHECK(range( 41) == Approx( 5.33488580347035));
        CHECK(range( 42) == Approx( 5.23066792452613));
        CHECK(range( 43) == Approx( 5.13197714973869));
        CHECK(range( 44) == Approx( 5.03844788869005));
        CHECK(range( 45) == Approx( 4.94974746830583));
        CHECK(range( 46) == Approx( 5.03844788869005));
        CHECK(range( 47) == Approx( 5.13197714973868));
        CHECK(range( 48) == Approx( 5.23066792452613));
        CHECK(range( 49) == Approx( 12.5876234368137));
        CHECK(range( 50) == Approx( 12.4457906148833));
        CHECK(range( 51) == Approx(  12.712125832526));
        CHECK(range( 52) == Approx(  19.669782333625));
        CHECK(range( 53) == Approx( 19.9396816934698));
        CHECK(range( 54) == Approx( 4.94427190999916));
        CHECK(range( 55) == Approx( 4.88309835504583));
        CHECK(range( 56) == Approx( 4.82487179401562));
        CHECK(range( 57) == Approx( 4.76945317134379));
        CHECK(range( 58) == Approx( 4.71769978699964));
        CHECK(range( 59) == Approx(  4.8540100660259));
        CHECK(range( 60) == Approx(                5));
        CHECK(range( 61) == Approx(  5.7167703393666));
        CHECK(range( 62) == Approx(  5.6628502534452));
        CHECK(range( 63) == Approx( 5.61163118817181));
        CHECK(range( 64) == Approx( 5.70293008176215));
        CHECK(range( 65) == Approx( 5.91550395788125));
        CHECK(range( 66) == Approx( 40.5667900369749));
        CHECK(range( 67) == Approx( 12.4931443401609));
        CHECK(range( 68) == Approx( 12.4031495407922));
        CHECK(range( 69) == Approx(  12.556926493314));
        CHECK(range( 70) == Approx( 33.5215998329912));
        CHECK(range( 71) == Approx( 33.7870883543296));
        CHECK(range( 72) == Approx( 29.9666733907906));
        CHECK(range( 73) == Approx(  28.233677425153));
        CHECK(range( 74) == Approx( 19.7656892813704));
        CHECK(range( 75) == Approx( 10.8703998943059));
        CHECK(range( 76) == Approx(  10.306136293499));
        CHECK(range( 77) == Approx( 10.2630410779339));
        CHECK(range( 78) == Approx( 10.2234059486503));
        CHECK(range( 79) == Approx( 7.64037521216411));
        CHECK(range( 80) == Approx( 7.61569958914309));
        CHECK(range( 81) == Approx( 7.59348844341002));
        CHECK(range( 82) == Approx( 7.57370679388964));
        CHECK(range( 83) == Approx( 15.1126473818827));
        CHECK(range( 84) == Approx( 15.0826241934527));
        CHECK(range( 85) == Approx( 51.6967216334824));
        CHECK(range( 86) == Approx( 18.0439541654611));
        CHECK(range( 87) == Approx( 7.51029259498441));
        CHECK(range( 88) == Approx( 7.50457158224116));
        CHECK(range( 89) == Approx( 7.50114246032931));
        CHECK(range( 90) == Approx(              7.5));
        CHECK(range( 91) == Approx( 22.5034273809879));
        CHECK(range( 92) == Approx( 40.5246865441023));
        CHECK(range( 93) == Approx( 48.0658726079002));
        CHECK(range( 94) == Approx( 26.5647102991511));
        CHECK(range( 95) == Approx( 33.6279645577021));
        CHECK(range( 96) == Approx( 22.1211821503974));
        CHECK(range( 97) == Approx( 57.4280600511544));
        CHECK(range( 98) == Approx( 30.7997409618179));
        CHECK(range( 99) == Approx( 6.58102331762202));
        CHECK(range(100) == Approx( 6.60027297725734));
        CHECK(range(101) == Approx( 6.62165851720889));
        CHECK(range(102) == Approx( 6.64521386662269));
        CHECK(range(103) == Approx( 25.6576026948348));
        CHECK(range(104) == Approx( 14.9438976255735));
        CHECK(range(105) == Approx( 13.9762284355361));
        CHECK(range(106) == Approx( 14.0440423841316));
        CHECK(range(107) == Approx( 32.4164444511016));
        CHECK(range(108) == Approx( 27.8637489423141));
        CHECK(range(109) == Approx( 1.53577674337861));
        CHECK(range(110) == Approx( 1.46190220008154));
        CHECK(range(111) == Approx( 1.39521405481267));
        CHECK(range(112) == Approx(   1.334733581277));
        CHECK(range(113) == Approx( 1.27965233262373));
        CHECK(range(114) == Approx( 1.22929666778712));
        CHECK(range(115) == Approx( 1.18310079157625));
        CHECK(range(116) == Approx( 1.14058601635243));
        CHECK(range(117) == Approx( 1.12232623763436));
        CHECK(range(118) == Approx( 1.13257005068904));
        CHECK(range(119) == Approx( 1.14335406787332));
        CHECK(range(120) == Approx( 1.15470053837925));
        CHECK(range(121) == Approx( 1.16663339721533));
        CHECK(range(122) == Approx(  1.1791784033621));
        CHECK(range(123) == Approx( 1.19236329283595));
        CHECK(range(124) == Approx(  1.2062179485039));
        CHECK(range(125) == Approx( 1.22077458876146));
        CHECK(range(126) == Approx( 1.23606797749979));
        CHECK(range(127) == Approx( 1.25213565815622));
        CHECK(range(128) == Approx( 1.26901821507258));
        CHECK(range(129) == Approx( 1.28675956589316));
        CHECK(range(130) == Approx( 1.30540728933228));
        CHECK(range(131) == Approx( 1.32501299334881));
        CHECK(range(132) == Approx( 1.34563272960638));
        CHECK(range(133) == Approx( 1.36732746109859));
        CHECK(range(134) == Approx( 1.39016359101668));
        CHECK(range(135) == Approx( 12.7279220613579));
        CHECK(range(136) == Approx( 9.73114513711676));
        CHECK(range(137) == Approx( 9.57129222769017));
        CHECK(range(138) == Approx( 9.71409757411996));
        CHECK(range(139) == Approx( 9.90764506358779));
        CHECK(range(140) == Approx( 9.13785102532595));
        CHECK(range(141) == Approx( 9.00731696125217));
        CHECK(range(142) == Approx(  8.9334808501551));
        CHECK(range(143) == Approx( 9.13902077617366));
        CHECK(range(144) == Approx( 18.7143177837449));
        CHECK(range(145) == Approx( 30.5193647190364));
        CHECK(range(146) == Approx( 18.6963782018105));
        CHECK(range(147) == Approx( 18.4816310389572));
        CHECK(range(148) == Approx(  24.173157268923));
        CHECK(range(149) == Approx( 24.2700503301294));
        CHECK(range(150) == Approx(               37));
        CHECK(range(151) == Approx( 22.2954043235297));
        CHECK(range(152) == Approx( 22.3655719159899));
        CHECK(range(153) == Approx( 26.4322711750232));
        CHECK(range(154) == Approx( 27.3740643924583));
        CHECK(range(155) == Approx( 17.1023577439186));
        CHECK(range(156) == Approx( 15.3249078990847));
        CHECK(range(157) == Approx( 15.3558279914847));
        CHECK(range(158) == Approx( 29.6597054236335));
        CHECK(range(159) == Approx( 21.4228998727406));
        CHECK(range(160) == Approx( 30.8611554018015));
        CHECK(range(161) == Approx( 35.3228650977083));
        CHECK(range(162) == Approx( 17.3491266999314));
        CHECK(range(163) == Approx( 14.6396845908201));
        CHECK(range(164) == Approx( 14.5641921020624));
        CHECK(range(165) == Approx( 15.0115046159462));
        CHECK(range(166) == Approx( 14.9438976255735));
        CHECK(range(167) == Approx( 15.3945616169009));
        CHECK(range(168) == Approx( 15.3351089229754));
        CHECK(range(169) == Approx( 9.67780860207453));
        CHECK(range(170) == Approx( 8.63112620102883));
        CHECK(range(171) == Approx( 8.60595356919803));
        CHECK(range(172) == Approx( 8.58353436640825));
        CHECK(range(173) == Approx( 8.56383351640021));
        CHECK(range(174) == Approx( 10.0550827956352));
        CHECK(range(175) == Approx( 12.5477479692918));
        CHECK(range(176) == Approx( 7.01709328656821));
        CHECK(range(177) == Approx( 7.00960642198545));
        CHECK(range(178) == Approx( 7.00426681009175));
        CHECK(range(179) == Approx( 7.00106629630735));
        CHECK(range(180) == Approx(                7));
        CHECK(range(181) == Approx( 1.50022849206586));
        CHECK(range(182) == Approx( 1.50091431644823));
        CHECK(range(183) == Approx( 1.50205851899688));
        CHECK(range(184) == Approx( 1.50366284712176));
        CHECK(range(185) == Approx( 1.50572975631502));
        CHECK(range(186) == Approx( 1.50826241934527));
        CHECK(range(187) == Approx( 1.51126473818827));
        CHECK(range(188) == Approx( 1.51474135877793));
        CHECK(range(189) == Approx( 1.51869768868201));
        CHECK(range(190) == Approx( 1.52313991782862));
        CHECK(range(191) == Approx( 1.52807504243282));
        CHECK(range(192) == Approx( 1.53351089229754));
        CHECK(range(193) == Approx( 1.53945616169009));
        CHECK(range(194) == Approx( 1.54592044402485));
        CHECK(range(195) == Approx( 1.55291427061512));
        CHECK(range(196) == Approx(  1.5604491537924));
        CHECK(range(197) == Approx( 1.56853763473072));
        CHECK(range(198) == Approx(  1.5771933363574));
        CHECK(range(199) == Approx( 5.28810340593335));
        CHECK(range(200) == Approx( 5.32088886237956));
        CHECK(range(201) == Approx( 4.18564216443801));
        CHECK(range(202) == Approx( 4.00420074383102));
        CHECK(range(203) == Approx( 3.83895699787118));
        CHECK(range(204) == Approx( 3.83122697477116));
        CHECK(range(205) == Approx( 3.86182271636872));
        CHECK(range(206) == Approx( 3.89410679166316));
        CHECK(range(207) == Approx( 3.92814183172027));
        CHECK(range(208) == Approx( 3.96399517741164));
        CHECK(range(209) == Approx( 4.00173923755662));
        CHECK(range(210) == Approx( 37.5277674973257));
        CHECK(range(211) == Approx(  8.7372181188466));
        CHECK(range(212) == Approx( 8.49185961659937));
        CHECK(range(213) == Approx( 6.42627460571832));
        CHECK(range(214) == Approx(  6.2590207748999));
        CHECK(range(215) == Approx( 6.10387294380728));
        CHECK(range(216) == Approx( 6.18033988749894));
        CHECK(range(217) == Approx( 6.26067829078112));
        CHECK(range(218) == Approx(  6.3450910753629));
        CHECK(range(219) == Approx(   6.356062916263));
        CHECK(range(220) == Approx( 6.22289530744165));
        CHECK(range(221) == Approx( 6.09701234682326));
        CHECK(range(222) == Approx( 6.05534728322869));
        CHECK(range(223) == Approx( 6.15297357494368));
        CHECK(range(224) == Approx( 6.25573615957505));
        CHECK(range(225) == Approx(  15.556349186104));
        CHECK(range(226) == Approx( 2.78032718203336));
        CHECK(range(227) == Approx( 2.73465492219719));
        CHECK(range(228) == Approx( 2.69126545921275));
        CHECK(range(229) == Approx( 2.65002598669762));
        CHECK(range(230) == Approx( 2.61081457866456));
        CHECK(range(231) == Approx( 2.57351913178634));
        CHECK(range(232) == Approx( 2.53803643014516));
        CHECK(range(233) == Approx( 2.50427131631245));
        CHECK(range(234) == Approx( 2.55195242505612));
        CHECK(range(235) == Approx( 2.61517019343165));
        CHECK(range(236) == Approx(  2.6824374749571));
        CHECK(range(237) == Approx( 2.75411768816499));
        CHECK(range(238) == Approx( 2.83061987219979));
        CHECK(range(239) == Approx( 2.91240603961553));
        CHECK(range(240) == Approx( 21.3619599600162));
        CHECK(range(241) == Approx( 9.14683254298656));
        CHECK(range(242) == Approx( 9.06056040551231));
        CHECK(range(243) == Approx( 8.97860990107489));
        CHECK(range(244) == Approx( 9.12468813081945));
        CHECK(range(245) == Approx( 11.0337791896249));
        CHECK(range(246) == Approx( 11.0636700100841));
        CHECK(range(247) == Approx( 14.0761756588609));
        CHECK(range(248) == Approx( 24.8062990815844));
        CHECK(range(249) == Approx( 19.2806098854665));
        CHECK(range(250) == Approx( 19.1551999045664));
        CHECK(range(251) == Approx( 21.6812239643268));
        CHECK(range(252) == Approx( 23.1321689332419));
        CHECK(range(253) == Approx( 8.36553405189719));
        CHECK(range(254) == Approx( 8.32239548689282));
        CHECK(range(255) == Approx( 8.28220944328067));
        CHECK(range(256) == Approx( 8.26713098887749));
        CHECK(range(257) == Approx( 38.4864040422522));
        CHECK(range(258) == Approx( 23.5138336818957));
        CHECK(range(259) == Approx( 23.5837937887553));
        CHECK(range(260) == Approx(  23.862525379315));
        CHECK(range(261) == Approx( 35.9425119654741));
        CHECK(range(262) == Approx( 29.7899133892992));
        CHECK(range(263) == Approx( 35.2628438910597));
        CHECK(range(264) == Approx( 13.0716076343257));
        CHECK(range(265) == Approx( 10.5401082942051));
        CHECK(range(266) == Approx( 10.5256399298523));
        CHECK(range(267) == Approx( 9.51303728698025));
        CHECK(range(268) == Approx( 9.50579067083881));
        CHECK(range(269) == Approx( 9.50144711641712));
        CHECK(range(270) == Approx(              9.5));
        CHECK(range(271) == Approx(0.500076164021954));
        CHECK(range(272) == Approx(0.500304772149411));
        CHECK(range(273) == Approx(0.500686172998961));
        CHECK(range(274) == Approx(0.501220949040586));
        CHECK(range(275) == Approx(0.501909918771674));
        CHECK(range(276) == Approx(0.502754139781759));
        CHECK(range(277) == Approx(0.503754912729425));
        CHECK(range(278) == Approx(0.504913786259309));
        CHECK(range(279) == Approx(0.506232562894002));
        CHECK(range(280) == Approx(0.507713305942873));
        CHECK(range(281) == Approx(0.509358347477608));
        CHECK(range(282) == Approx(0.511170297432515));
        CHECK(range(283) == Approx(0.513152053896695));
        CHECK(range(284) == Approx( 0.51530681467495));
        CHECK(range(285) == Approx(0.517638090205043));
        CHECK(range(286) == Approx(0.520149717930803));
        CHECK(range(287) == Approx(0.522845878243575));
        CHECK(range(288) == Approx(0.525731112119134));
        CHECK(range(289) == Approx(0.528810340593335));
        CHECK(range(290) == Approx(0.532088886237954));
        CHECK(range(291) == Approx(0.535572496818512));
        CHECK(range(292) == Approx(0.539267371338794));
        CHECK(range(293) == Approx(0.543180188702646));
        CHECK(range(294) == Approx(0.547318139253023));
        CHECK(range(295) == Approx(0.551688959481244));
        CHECK(range(296) == Approx(0.556300970237592));
        CHECK(range(297) == Approx(0.561163118817178));
        CHECK(range(298) == Approx(0.566285025344521));
        CHECK(range(299) == Approx(0.571677033936662));
        CHECK(range(300) == Approx(0.577350269189625));
        CHECK(range(301) == Approx(0.583316698607668));
        CHECK(range(302) == Approx(0.589589201681048));
        CHECK(range(303) == Approx(0.596181646417973));
        CHECK(range(304) == Approx(0.603108974251952));
        CHECK(range(305) == Approx(0.610387294380724));
        CHECK(range(306) == Approx(0.618033988749893));
        CHECK(range(307) == Approx(0.626067829078112));
        CHECK(range(308) == Approx(0.634509107536293));
        CHECK(range(309) == Approx(0.643379782946582));
        CHECK(range(310) == Approx(0.652703644666138));
        CHECK(range(311) == Approx(0.662506496674405));
        CHECK(range(312) == Approx(0.672816364803185));
        CHECK(range(313) == Approx(  0.6836637305493));
        CHECK(range(314) == Approx(0.695081795508336));
        CHECK(range(315) == Approx( 12.0208152801713));
        CHECK(range(316) == Approx( 9.73114513711676));
        CHECK(range(317) == Approx( 9.57129222769017));
        CHECK(range(318) == Approx( 9.71409757411995));
        CHECK(range(319) == Approx( 9.90764506358779));
        CHECK(range(320) == Approx( 10.1122048745927));
        CHECK(range(321) == Approx( 15.8901572906575));
        CHECK(range(322) == Approx( 17.0548270775688));
        CHECK(range(323) == Approx( 13.7734922397185));
        CHECK(range(324) == Approx( 9.27050983124843));
        CHECK(range(325) == Approx( 9.15580941571092));
        CHECK(range(326) == Approx( 9.04663461377929));
        CHECK(range(327) == Approx( 9.18039229388332));
        CHECK(range(328) == Approx(  11.791784033621));
        CHECK(range(329) == Approx( 11.6663339721533));
        CHECK(range(330) == Approx(               12));
        CHECK(range(331) == Approx( 13.4073247075775));
        CHECK(range(332) == Approx( 18.1054629796109));
        CHECK(range(333) == Approx( 19.0795460397841));
        CHECK(range(334) == Approx( 10.0134174642767));
        CHECK(range(335) == Approx( 9.93040127066243));
        CHECK(range(336) == Approx( 9.85172650655442));
        CHECK(range(337) == Approx( 10.2372186609898));
        CHECK(range(338) == Approx( 13.3473358127701));
        CHECK(range(339) == Approx( 14.9960299109184));
        CHECK(range(340) == Approx( 14.8984888146628));
        CHECK(range(341) == Approx( 23.2676549861068));
        CHECK(range(342) == Approx( 11.0403533545018));
        CHECK(range(343) == Approx( 10.9797634431151));
        CHECK(range(344) == Approx( 5.20149717930801));
        CHECK(range(345) == Approx( 5.17638090205042));
        CHECK(range(346) == Approx( 5.15306814674949));
        CHECK(range(347) == Approx( 5.13152053896696));
        CHECK(range(348) == Approx( 5.11170297432515));
        CHECK(range(349) == Approx( 5.24084306416782));
        CHECK(range(350) == Approx( 8.63815572471545));
        CHECK(range(351) == Approx( 25.5698128859986));
        CHECK(range(352) == Approx( 10.6031895114455));
        CHECK(range(353) == Approx( 10.5788531673179));
        CHECK(range(354) == Approx( 10.5578369354169));
        CHECK(range(355) == Approx( 29.6126852075288));
        CHECK(range(356) == Approx( 12.0293027769741));
        CHECK(range(357) == Approx( 12.0164681519751));
        CHECK(range(358) == Approx( 12.5076193037353));
        CHECK(range(359) == Approx( 12.5019041005488));
    }   
}