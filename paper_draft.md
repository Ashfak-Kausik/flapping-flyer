
%% 
%% Copyright 2007-2020 Elsevier Ltd
%% 
%% This file is part of the 'Elsarticle Bundle'.
%% ---------------------------------------------
%% 
%% It may be distributed under the conditions of the LaTeX Project Public
%% License, either version 1.2 of this license or (at your option) any
%% later version.  The latest version of this license is in
%%    http://www.latex-project.org/lppl.txt
%% and version 1.2 or later is part of all distributions of LaTeX
%% version 1999/12/01 or later.
%% 
%% The list of all files belonging to the 'Elsarticle Bundle' is
%% given in the file `manifest.txt'.
%% 

%% Template article for Elsevier's document class `elsarticle'
%% with numbered style bibliographic references
%% SP 2008/03/01
%%
%% $Id: elsarticle-template-num.tex 190 2020-11-23 11:12:32Z rishi $
%%
%\documentclass[preprint,12pt]{elsarticle}


%\documentclass[final,twocolumn, 3p]{elsarticle}

%% Use the option review to obtain double line spacing
%% \documentclass[authoryear,preprint,review,12pt]{elsarticle}

%% Use the options 1p,twocolumn; 3p; 3p,twocolumn; 5p; or 5p,twocolumn
%% for a journal layout:
%% \documentclass[final,1p,times]{elsarticle}
%% \documentclass[final,1p,times,twocolumn]{elsarticle}
%% \documentclass[final,3p,times]{elsarticle}
%% \documentclass[final,3p,times,twocolumn]{elsarticle}
%% \documentclass[final,5p,times]{elsarticle}
%% \documentclass[final,5p,times,twocolumn]{elsarticle}

%% For including figures, graphicx.sty has been loaded in
%% elsarticle.cls. If you prefer to use the old commands
%% please give \usepackage{epsfig}


%% The amsthm package provides extended theorem environments
%% \usepackage{amsthm}

%% 
%% Copyright 2007-2020 Elsevier Ltd
%% 
%% This file is part of the 'Elsarticle Bundle'.
%% ---------------------------------------------
%% 
%% It may be distributed under the conditions of the LaTeX Project Public
%% License, either version 1.2 of this license or (at your option) any
%% later version.  The latest version of this license is in
%%    http://www.latex-project.org/lppl.txt
%% and version 1.2 or later is part of all distributions of LaTeX
%% version 1999/12/01 or later.
%% 
%% The list of all files belonging to the 'Elsarticle Bundle' is
%% given in the file `manifest.txt'.
%% 

%% Template article for Elsevier's document class `elsarticle'
%% with numbered style bibliographic references
%% SP 2008/03/01
%%

%% $Id: elsarticle-template-num.tex 190 2020-11-23 11:12:32Z rishi $
%%
%\documentclass[preprint,5pt]{elsarticle}

\documentclass[final,5p,times]{elsarticle}

%% Use the option review to obtain double line spacing
%% \documentclass[authoryear,preprint,review,12pt]{elsarticle}

%% Use the options 1p,twocolumn; 3p; 3p,twocolumn; 5p; or 5p,twocolumn
%% for a journal layout:
%% \documentclass[final,1p,times]{elsarticle}
%% \documentclass[final,1p,times,twocolumn]{elsarticle}
%% \documentclass[final,3p,times]{elsarticle}
%% \documentclass[final,3p,times,twocolumn]{elsarticle}
%% \documentclass[final,5p,times]{elsarticle}
%% \documentclass[final,5p,times,twocolumn]{elsarticle}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Added packages and macros
%===================================
% \graphicspath{{figures/}}
% \usepackage{pifont}
% \newcommand{\cmark}{\ding{51}}

\usepackage{makecell}
\usepackage{pifont}
\usepackage{amssymb,amsmath, latexsym, bm, amsfonts}
\usepackage{algorithm, algcompatible}
\usepackage{textcomp}
\usepackage{xcolor}
\usepackage{balance}
\usepackage{subcaption}
\usepackage{gensymb}
\usepackage{multirow, array, ragged2e, lscape, eqparbox, xspace, setspace}
\usepackage{array, rotating,  threeparttable, booktabs, longtable, tabularx,tabulary}
\usepackage{natbib, url}
\usepackage{lipsum}
\usepackage{enumitem}
\setlist[itemize]{topsep=2pt, left=0pt, itemsep=2pt, parsep=0pt}
\usepackage{algpseudocode}

\usepackage{tikz}
\usetikzlibrary{arrows.meta, positioning, calc, fit, backgrounds}

\usepackage{flushend}
\makeatletter
\newcommand\singlelipsum[1]{%
  \begingroup\let\lips@par\relax\csname lipsum@\@roman{#1}\endcsname
\endgroup }
\makeatother



\def\BibTeX{{\rm B\kern-.05em{\sc i\kern-.025em b}\kern-.08em
		T\kern-.1667em\lower.7ex\hbox{E}\kern-.125emX}}


\pdfminorversion=6
%\usepackage{cmap, colortbl}

\usepackage{hyperref}	%pageanchor=false,
\hypersetup{colorlinks, breaklinks, citecolor=blue, urlcolor=black, linkcolor=black}
% to display URLs in blue roman font according to Springer's eBook style:
% \renewcommand\UrlFont{\colour{blue}\rmfamily}
\pdfsuppresswarningpagegroup=1
% \newcommand*{\email}[1]{\normalsize\texttt{\href{mailto:#1}{#1}}\par}


\pdfminorversion=6
%\makeatother
\usepackage{hyphenat}
\def\hyph{-\penalty0\hskip0pt\relax}
\sloppy

%\usepackage{enumitem}
\newcommand{\func}[1]{\ensuremath{{\tt #1}\left(\cdot\right)}}
\newcommand{\funcc}[2]{\ensuremath{{\tt #1}\left({#2}\right)}}
\renewcommand{\vec}[1]{\mathbf{#1}}
\newcommand*{\email}[1]{\normalsize\texttt{\href{mailto:#1}{#1}}\par}

\newcolumntype{L}[1]{>{\raggedright\arraybackslash}p{#1}}
\newcolumntype{C}[1]{>{\centering\arraybackslash}p{#1}}
\newcolumntype{R}[1]{>{\raggedleft\arraybackslash}p{#1}}
\newcolumntype{Y}{>{\raggedright\arraybackslash}X}
% \setlength\extrarowheight{10pt}
%==============================
% \newcolumntype{b}{X}
% \newcolumntype{s}{>{\hsize=.5\hsize}X}
\newcommand{\heading}[1]{\multicolumn{1}{c}{#1}}
\makeatletter
\DeclareRobustCommand\onedot{\futurelet\@let@token\@onedot}
\def\@onedot{\ifx\@let@token.\else.\null\fi\xspace}
\def\eg{\emph{e.g}\onedot} \def\Eg{\emph{E.g}\onedot}
\def\ie{\emph{i.e}\onedot} \def\Ie{\emph{I.e}\onedot}
\def\cf{\emph{c.f}\onedot} \def\Cf{\emph{C.f}\onedot}
\def\etc{\emph{etc}\onedot} \def\vs{\emph{vs}\onedot}
\def\wrt{w.r.t\onedot} \def\dof{d.o.f\onedot}
\def\etal{\emph{et al}\onedot}
\makeatother

%% ============================================
%% DOCUMENT BEGINS
%% ============================================

\begin{document}

\begin{frontmatter}

\title{Reactive Aerodynamic Sensing and Navigation for a Bio-Inspired Flapping-Wing Micro Air Vehicle in Perception-Denied Confined Spaces}

\author[inst1]{MD Ashfakul Karim Kausik\corref{cor1}}
\ead{ashfakkausik65@gmail.com}

\author[inst2]{Naymul Siddque}
\ead{tahsin92refat@gmail.com}

\author[inst3]{Md Sayem Hossain Bhuiyan}
\ead{alvi676@gmail.com}

\author[inst3]{Tareque Bashar Ovi}
\ead{ovitareque@gmail.com}

\author[inst4]{Md Ahsan Kabir\corref{cor1}}
\ead{ahsan@eece.mist.ac.bd}

\cortext[cor1]{Corresponding authors}

\affiliation[inst1]{organization={Military Institute of Science and Technology},
                    addressline={Department of Industrial Engineering},
                    city={Dhaka},
                    country={Bangladesh}}

\affiliation[inst2]{organization={Military Institute of Science and Technology},
                    addressline={Department of Computer Science},
                    city={Dhaka},
                    country={Bangladesh}}

\affiliation[inst3]{organization={Khulna University of Engineering and Technology},
                    addressline={Department of Electronics and Communication Engineering},
                    city={Khulna},
                    country={Bangladesh}}

\affiliation[inst4]{organization={Military Institute of Science and Technology},
                    addressline={Department of Electrical, Electronic and Communication Engineering},
                    city={Dhaka},
                    country={Bangladesh}}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ABSTRACT (Full Width Automatically)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\begin{abstract}
Robotic access to confined, hazardous, GPS-denied spaces is limited by a sensing dilemma: platforms small enough to enter cannot easily carry sensors, and vision and positioning fail in darkness and dust. This paper investigates, in simulation, whether a $77.4$~mg bio-inspired flapping-wing micro air vehicle can navigate and inspect such spaces reactively, using only a classical control system, with no learned policy, map, or external positioning. We show that the ground-effect interaction between a flapping wing and a nearby wall induces a roll disturbance that the vehicle's own disturbance observer already estimates, yielding a ``wing-wash'' proximity signal at no added sensing cost, with a usable range of $8$ to $40$~mm within the adopted model. Combined with forward-facing feelers, a reactive controller detects isolated wall openings from about $12$~mm and, across $40$ randomized multi-turn courses, reaches the goal on $35$ and does so without a wingtip strike on $30$. Critically, a controlled, pre-registered ablation attributes this competence to the feeler channel: removing the aerodynamic signal produces no detectable change in strike-free completion (paired difference $+1.0$ percentage points, $95\%$ confidence interval $[-5.0, +6.5]$, $p=0.83$), although formal equivalence within a $\pm 5$ point margin is not established, while removing the feelers collapses the task. We report a dissociation between what a signal \emph{is} and what it \emph{does}, note that it is observed only against an idealized tactile channel, and specify the experiments that would test it under realistic conditions.
\end{abstract}

\begin{keyword}
Flapping-wing micro air vehicle \sep Bio-inspired robotics \sep Reactive navigation \sep Aerodynamic proximity sensing \sep Ground effect \sep Confined-space inspection \sep GPS-denied navigation \sep Disturbance observer
\end{keyword}

\end{frontmatter}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN PAPER (TWO COLUMN AUTOMATICALLY STARTS HERE)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Introduction}
\label{sec:introduction}

Small flying robots are often most needed in places where they can sense the least. The interior voids of a collapsed building, the ducts and crawlspaces of an industrial plant, and the damaged compartments of a containment structure are all confined, cluttered, and frequently dark and dust-laden, and they usually lie beyond the reach of satellite positioning. These conditions degrade or disable the sensors on which aerial autonomy normally depends, such as cameras and laser scanners, exactly where a robot would be most useful. At the same time, only a very small vehicle can enter such spaces, and a small vehicle has little mass and power budget to carry sensors at all. The result is a tension between the size needed to enter a confined space and the sensing needed to operate within it \cite{floreano2015science}.

Bio-inspired flapping-wing micro air vehicles (FWMAVs) are attractive for this setting. At insect scale they are very light, and their flapping wake carries far less momentum than a rotor of equivalent lift, so they disturb loose dust and contaminants less than a small rotorcraft would \cite{floreano2015science}. Over the past two decades such vehicles have progressed from the first at-scale robotic insect to take off \cite{wood2008first}, to the first controlled flight of an insect-scale vehicle \cite{ma2013controlled}, and to untethered, power-autonomous flight \cite{jafferis2019untethered}. A recurring and still largely open question, however, is how such a vehicle should perceive a confined space when it cannot carry a conventional sensor payload. One idea, inspired by biology, is that a flapping-wing vehicle might sense nearby surfaces through the same aerodynamics that it already uses to fly, rather than through an added sensor.

\subsection{Background and Motivation}
\label{sec:background}

A wing operating close to a surface experiences a change in the forces it produces, an effect long studied in rotorcraft and known as the aerodynamic ground effect \cite{cheeseman1955}. For a flapping vehicle, when one wing is nearer a wall than the other, this effect is asymmetric, and the difference produces a small rolling moment that depends on the distance to the wall. In principle this moment is available to the vehicle without any dedicated sensor, because a flight controller that already estimates external disturbances in order to reject them is, in effect, already measuring it. This study refers to the resulting signal as the ``wing-wash'' signal. The idea that a flying animal can perceive surfaces from its own induced airflow is well supported in biology: nocturnal mosquitoes have been shown to divert away from walls and ground using mechanosensory cues from their self-generated flow, and this principle has been reproduced on a small quadcopter fitted with pressure probes \cite{nakata2020}. Tactile sensing provides a second, complementary biological precedent, as the antenna-based wall-following of the cockroach has been formalized and transferred to mobile robots \cite{lee2008templates}.

The motivation for studying this aerodynamic sense carefully, rather than assuming it to be useful, comes from the one prior hardware study that attempted something closely related. In that work, a flapping-wing vehicle intended to navigate tight spaces was found, on the physical platform, to produce no aerodynamic wall signal usable for control, and the authors used direct wing contact for sensing instead \cite{actingIsSeeing2019}. This does not settle the question, since it concerns one platform and one set of conditions, but it does caution against treating the usefulness of an aerodynamic proximity sense as obvious. It also points to a distinction that is easy to overlook: a signal may be accurately measurable and yet contribute little to a task, if another sensing channel already supplies what the task needs. To the authors' knowledge, this distinction has not been examined directly for a flapping-wing vehicle, and it is the central concern of the present study.

\subsection{Problem Statement}
\label{sec:problem}

A flapping-wing vehicle operating in a confined, perception-denied space must, at minimum, keep itself centred in a passage, negotiate corners and junctions, and recognize openings in the walls, all without vision, without external positioning, and without a sensor payload that its mass budget cannot support. The wing-wash signal described above is present on any flapping-wing vehicle by virtue of its propulsion, and a lightweight forward-facing feeler is a well-established complementary cue \cite{lee2008templates}. Two questions therefore arise, and they are distinct. The first is whether the wall-induced aerodynamic disturbance can be characterized as a proximity sensing modality in its own right, with a defined range, resolution, and dynamic response. The second, which does not follow from the first, is whether that signal is of practical value to a vehicle that also carries a tactile feeler, since a measurement may be accurate yet redundant. A system evaluated only in its complete configuration cannot answer the second question, because the contribution of any one channel is then hidden. This study addresses both questions within a physics-based simulation, and treats the second as the more consequential of the two.

\subsection{Aims, Goals, and Objectives}
\label{sec:aims}

The overall aim of this study is to determine what a flapping wing's near-wall aerodynamic interaction can and cannot contribute to reactive navigation and inspection in confined, perception-denied spaces, using only a classical control system, and to report the result without overstating it. The study is conducted entirely in simulation; no hardware experiment is performed, and the transfer of any result to a physical vehicle is treated as an open question rather than a demonstrated fact. To pursue this aim, the specific objectives are as follows.

\begin{itemize}
    \item To model a mosquito-scale flapping-wing vehicle and its near-wall aerodynamics within a rigid-body physics engine, and to establish a control-only flight system, based on a linear-quadratic-Gaussian regulator with a disturbance observer, that achieves stable hovering and cruising flight.
    \item To characterize the wall-induced roll disturbance as a proximity sensing modality, reporting its effective range, resolution, observer-estimation fidelity, and two-wall corridor behaviour, and to state clearly which of these properties are assumed by the model, implied by it, verified in the implementation, or emergent from closed-loop behaviour.
    \item To build a purely reactive controller that combines this aerodynamic signal with forward-facing feelers to follow corridors, negotiate corners and forks, and distinguish a wall opening to be inspected from a passage to be followed.
    \item To evaluate the resulting system on the task of detecting and localizing wall openings, both on crafted courses and across a randomized ensemble of passages, under a sensor-noise model.
    \item To attribute the observed competence to its actual sensory source through a controlled ablation, removing each sensing channel in turn on identical courses, and to test whether any operating regime exists in which the aerodynamic channel confers a measurable advantage.
\end{itemize}

\subsection{Research Contributions}
\label{sec:contributions}

This study makes the following contributions, the last of which the authors regard as the most consequential and the least anticipated. In reporting them, properties that are consequences of the modelling choices are labelled as such, so that they are not mistaken for independently established physical facts.

\begin{itemize}
    \item \textbf{A control-only flapping-wing platform for reactive confined-space navigation:} The study develops a 77.4~mg simulated flapping-wing vehicle, of 25.2~mm tip-to-tip aerodynamic span, that achieves stable flight and reactively negotiates confined corridors using a classical control architecture alone, with no learned policy, planner, map, or external positioning.
    \item \textbf{Characterization of a ``free'' aerodynamic proximity signal:} The wall-induced roll disturbance, recovered from the vehicle's own disturbance observer at no additional sensing cost, is characterized as a proximity instrument with a usable range of approximately 8 to 40~mm and a sub-millimetre noise-equivalent distance under the adopted noise model, together with its observer gain and two-wall response. Each reported property is classified by its evidential status, so that a model-encoded property, such as the exact two-wall superposition, is not presented as a discovered aerodynamic fact.
    \item \textbf{A reactive gap-versus-path inspection capability, evaluated across a randomized ensemble:} The integrated system detects isolated wall openings from about 12~mm and localizes those up to 35~mm long to within about a millimetre in a straight corridor; on multi-turn courses the mean localization error is 7.9~mm. The system is evaluated not on a single hand-designed route but across 40 independently generated multi-turn courses, with navigation and inspection outcomes reported in both their conditional and their end-to-end forms.
    \item \textbf{A controlled, statistically grounded ablation establishing a measurability-versus-utility dissociation:} By removing each sensing channel in turn on geometrically identical courses, and by subjecting the central comparison to a pre-registered paired analysis, the study detects no contribution of the aerodynamic channel to the primary task, with a confidence interval that bounds any benefit at about 6.5 percentage points but does not establish formal equivalence, while removing the tactile channel more than halves goal-reaching. It further tests, and does not confirm, several regimes in which the aerodynamic channel might have been expected to help. The honest delineation of where a measurable sense does and does not earn its place is the central finding of the work.
\end{itemize}

The remainder of this paper is organized as follows. Section~\ref{sec:relatedwork} reviews the related literature and identifies the gap addressed. Section~\ref{sec:methodology} presents the vehicle model, the aerodynamic and near-wall models, the control and estimation architecture, the reactive navigation layer, and the experimental and statistical design. Section~\ref{sec:results} reports the characterization of the aerodynamic signal, the navigation and inspection outcomes, and the sensory-channel ablation with its statistical analysis. Section~\ref{sec:discussion} interprets the findings and states their limitations and the directions for further study, and Section~\ref{sec:conclusion} concludes.

\section{Related Work}
\label{sec:relatedwork}

This section reviews the prior work that the present study builds upon and against which it is positioned. It covers, in turn, the development and control of bio-inspired flapping-wing micro air vehicles, the aerodynamics of wings operating near surfaces, and the use of self-induced airflow and of tactile contact as sensing cues. Table~\ref{tab:litreview} summarizes the most directly relevant works, what each contributed, and how the present study relates to it, and the discussion that follows treats each strand in turn. The intent is not an exhaustive survey but a focused positioning of the contribution.

\begin{table*}[t]
\centering
\caption{Summary of representative prior work and its relation to the present study.}
\label{tab:litreview}
\footnotesize
\renewcommand{\arraystretch}{1.3}
\begin{tabular}{@{}p{2.0cm} p{1.1cm} p{4.4cm} p{4.4cm} p{4.4cm}@{}}
\toprule
\textbf{Research Domain} & \textbf{Ref.} & \textbf{Research Scope} & \textbf{Key Contribution} & \textbf{Gap and Relevance} \\
\midrule
FWMAV flight &
\cite{wood2008first} &
Designed an at-scale piezoelectric robotic insect and achieved tethered takeoff. &
First demonstration that an insect-scale flapping-wing vehicle can generate lift to take off. &
Establishes the platform class; this study models a comparable insect-scale vehicle in simulation. \\

FWMAV control &
\cite{ma2013controlled} &
Added feedback control to an insect-scale flapping-wing robot for stabilized flight. &
First controlled, hovering and maneuvering flight of an insect-scale robot. &
Motivates the control-only approach; this study uses a classical LQG regulator rather than a tracked policy. \\

FWMAV autonomy &
\cite{jafferis2019untethered} &
Developed an untethered, onboard-powered insect-scale flapping-wing vehicle. &
First untethered flight of an insect-scale vehicle, removing the tether constraint. &
Supports the feasibility of payload-free operation that this study assumes for the vehicle. \\

FWMAV design &
\cite{phan2017} &
Built a 21~g tailless flapping-wing vehicle with angular-rate feedback. &
Stable hovering of a tailless design using simple onboard feedback. &
Confirms that feedback stabilization, not learning, suffices for stable flapping flight. \\

FWMAV control &
\cite{karasek2018} &
Developed the 29~g DelFly Nimble, controlled entirely through wing-motion adjustment. &
Agile free flight and a biological finding on torque coupling in banked turns. &
Example of perception directed at flight control rather than environmental sensing. \\

Near-wall aerodynamics &
\cite{cheeseman1955} &
Derived the classical rotor ground-effect relation for a lifting surface near a surface. &
Foundational analytic model of distance-dependent lift modification near a boundary. &
Provides the functional form adopted for the near-wall law in this study. \\

Near-wall aerodynamics &
\cite{maeda2013} &
Computationally studied fruit-fly hovering in ground effect in three dimensions. &
Showed distance-dependent force modification for an insect-scale flapping wing. &
Supports the physical plausibility of the modelled wall interaction at this scale. \\

Near-wall aerodynamics &
\cite{lua2016} &
Studied ground effect on three-dimensional hovering wings experimentally and numerically. &
Identified three force regimes (enhancement, reduction, recovery) with clearance. &
Consistent with the non-monotonic, saturating response observed in this study. \\

Aerodynamic sensing &
\cite{nakata2020} &
Showed mosquitoes sense surfaces from self-induced airflow and built a quadcopter detector. &
Demonstrated aerodynamic surface sensing on a real flyer and a rotorcraft. &
Establishes the idea is real and useful; this study asks whether it helps once a tactile cue exists. \\

Flapping-wing sensing &
\cite{actingIsSeeing2019} &
Used wing-loading current to sense ground, walls, and obstacles on a flapping robot. &
Vision-free tight-space navigation using actuator-loading cues. &
Reported the wall cue undetectable on hardware and used contact; central motivation for this study. \\

Tactile sensing &
\cite{lee2008templates} &
Formalized antenna-based wall-following in cockroaches and transferred it to robots. &
Template models for lightweight, vision-free tactile wall-following. &
Basis for the forward-facing feeler channel used alongside the aerodynamic signal. \\

Disturbance estimation &
\cite{chen2016disturbance} &
Surveyed disturbance-observer-based control and related estimation methods. &
Comprehensive overview of estimating and compensating external disturbances. &
Underlies reinterpreting the controller's disturbance estimate as an environmental cue. \\

\bottomrule
\end{tabular}
\end{table*}

\subsection{Flapping-Wing Micro Air Vehicles and Their Control}
\label{sec:rw_fwmav}

Research on flapping-wing micro air vehicles (FWMAVs) has advanced considerably over the past two decades. At insect scale, the Harvard RoboBee demonstrated the first takeoff of an at-scale robotic insect \cite{wood2008first}, followed by the first controlled flight of an insect-scale vehicle \cite{ma2013controlled} and, later, untethered and power-autonomous flight \cite{jafferis2019untethered}. At larger scales, motor-driven designs such as the tailless KUBeetle achieved stable hovering flight using wing-rotation modulation and simple feedback control \cite{phan2017}, and the DelFly Nimble, a 29~g tailless flapping-wing robot of 33~cm wingspan, demonstrated agile free flight controlled entirely through adjustments of its flapping wings \cite{karasek2018}. A common theme across these platforms is that the vehicle is intrinsically unstable and underactuated, and that stable flight requires active feedback control under severe mass and power constraints \cite{phan2017, karasek2018}. In nearly all cases the onboard perception, where present, has been directed toward stabilizing flight, and, where the payload allowed, toward carrying a camera, rather than toward sensing the surrounding environment through the flapping aerodynamics themselves. The present study adopts a classical linear-quadratic-Gaussian controller with a disturbance observer, in the same spirit of feedback stabilization, but examines whether the controller's own disturbance estimate can additionally serve as an environmental sense.

\subsection{Aerodynamics of Flapping Wings Near Surfaces}
\label{sec:rw_groundeffect}

The interaction between a lifting surface and a nearby boundary, known as the aerodynamic ground effect, is a classical topic in rotorcraft aerodynamics \cite{cheeseman1955}, and it has been studied specifically for flapping-wing flight at the low Reynolds numbers characteristic of insects. Computational studies of fruit-fly hovering have shown that a nearby ground alters the produced forces in a manner that depends on the distance to the surface \cite{maeda2013}. A recurring finding in this literature is that, as the wing approaches a surface, the cycle-averaged forces do not change monotonically but instead pass through three regimes with decreasing clearance, namely force enhancement, force reduction, and force recovery \cite{lua2016}. This three-regime behaviour is relevant to the present work, because it is consistent with the non-monotonic, saturating response that is observed when the modelled signal is characterized at very close range, as reported in Section~\ref{sec:results_sensor}. Within the FWMAV community, the near-wall interaction has most often been treated as a disturbance to be compensated rather than as a signal to be exploited. The present study adopts a simple model of this interaction, stated explicitly in Section~\ref{sec:method_wingwash}, and asks whether its asymmetric, left-right form can serve as a lateral proximity cue.

\subsection{Aerodynamic and Tactile Sensing of Surfaces}
\label{sec:rw_sensing}

The idea that a flying agent can perceive nearby surfaces from its own induced airflow has strong support in biology and has recently been demonstrated in engineering. Nakata et al. showed that nocturnal mosquitoes divert away from walls and ground using mechanosensory cues derived from the modification of their self-generated airflow near a surface, and the authors reproduced this principle on a small quadcopter fitted with pressure probes, achieving obstacle detection at a distance of a few rotor diameters \cite{nakata2020}. For flapping-wing vehicles specifically, Tu et al. presented a motor-driven hummingbird robot that used the wing-loading signature in its actuator current to detect ground, walls, and obstacles, and navigated a narrow corridor without vision \cite{actingIsSeeing2019}. Notably, that study reported that a usable aerodynamic \emph{wall} signal could not be detected on their platform, attributed to weak spanwise airflow, and the authors therefore used physical wing contact to sense walls instead \cite{actingIsSeeing2019}. This outcome is directly relevant to the present work, since it indicates that the usefulness of an aerodynamic wall cue cannot be assumed and depends on the platform and conditions. A complementary and well-established sensing modality is tactile contact: the antenna-based wall-following of the cockroach has been formalized into template models and transferred to mobile robots \cite{lee2008templates}, providing a lightweight, vision-free means of detecting surfaces along the direction of travel. The present study draws on both lines, treating the observer-estimated wall disturbance as a candidate aerodynamic proximity cue and a set of forward-facing feelers as a tactile cue, and it reinterprets a quantity already estimated for control, in the manner of disturbance-observer-based methods \cite{chen2016disturbance}, as information about the environment.

\subsection{Summary and Research Gap}
\label{sec:rw_gap}

The literature reviewed above establishes three points. First, flapping-wing vehicles are increasingly capable but have directed their onboard perception primarily toward flight stabilization rather than environmental sensing \cite{phan2017, karasek2018}. Second, the aerodynamic interaction between a wing and a nearby surface is physically established and distance-dependent, and perceiving surfaces from self-induced airflow has been demonstrated on real flyers and on a quadcopter \cite{nakata2020, lua2016}. Third, however, the one prior attempt to use such a cue specifically for \emph{flapping-wing wall} sensing found it undetectable on hardware and resorted to contact sensing instead \cite{actingIsSeeing2019}. Taken together, these points indicate that the existence of a near-wall aerodynamic effect does not by itself establish that it provides a useful navigational signal, particularly when a vehicle also carries a simple tactile cue. To the authors' knowledge, the question of whether such an aerodynamic cue contributes measurably to confined-space navigation, as distinct from whether it can be measured at all, has not been examined directly for a flapping-wing vehicle. The present study addresses this gap: it characterizes the wall-induced disturbance as a proximity signal, integrates it with a tactile feeler in a reactive controller, and then, by controlled ablation, attributes the resulting competence to its actual sensory source. In doing so it distinguishes what a signal \emph{is} from what it \emph{does}, a distinction that a system evaluated only in its complete configuration cannot make.

\section{Methodology}
\label{sec:methodology}

This study follows a simulation-based experimental design, in which a mosquito-scale flapping-wing vehicle, its near-wall aerodynamics, and its control and sensing architecture are modelled within a rigid-body physics engine and then subjected to a sequence of controlled numerical experiments. Figure~\ref{fig:workflow} presents the overall workflow, from the physical model through the control architecture and the sensing it exposes, to the evaluation and the controlled ablation that attributes the observed competence to its source; the subsections that follow describe each stage in turn.

\begin{figure*}[t]
\centering
\begin{tikzpicture}[
  font=\footnotesize,
  >={Stealth[length=2mm]},
  blk/.style={draw, rounded corners=2pt, align=center, minimum height=9mm,
              text width=24mm, inner sep=3pt},
  key/.style={draw, rounded corners=2pt, align=center, minimum height=9mm,
              text width=24mm, inner sep=3pt, fill=black!7, very thick},
  grp/.style={draw, rounded corners=3pt, align=center, minimum height=8mm,
              text width=27mm, inner sep=3pt, fill=black!3},
  arr/.style={->, thick},
  darr/.style={->, thick, dashed}
]
% row 1: platform
\node[blk] (plant) {MuJoCo rigid-body\\plant (Sec.~\ref{sec:method_vehicle})};
\node[blk, right=12mm of plant] (aero) {Blade-element\\aerodynamics (Sec.~\ref{sec:method_aero})};
\node[blk, right=12mm of aero] (kappa) {Near-wall law\\$\kappa(d)$ (Sec.~\ref{sec:method_wingwash})};
\draw[arr] (aero) -- (plant);
\draw[arr] (kappa) -- (aero);
% row 2: control and sensing
\node[blk, below=12mm of plant] (sensors) {Noisy sensors\\(Sec.~\ref{sec:method_noise})};
\node[key, right=12mm of sensors] (kf) {Observer:\\state $\hat{\mathbf{x}}$, disturbance $\hat{\delta}$};
\node[blk, right=12mm of kf] (nav) {Reactive navigation\\(Sec.~\ref{sec:method_nav})};
\draw[arr] (plant) -- (sensors);
\draw[arr] (sensors) -- (kf);
\node[blk, below=10mm of kf] (lqg) {LQG regulator\\(Sec.~\ref{sec:method_control})};
\draw[arr] (kf) -- node[right, pos=0.4] {$\hat{\mathbf{x}}$} (lqg);
\draw[arr] (kf) -- node[above] {$\hat{\delta}$ (wing-wash)} (nav);
\draw[arr] (nav) |- (lqg.east);
\draw[arr] (lqg) -| (plant.south);
% row 3: evaluation
\node[grp, below=20mm of sensors] (e1) {Signal\\characterization};
\node[grp, right=10mm of e1] (e2) {Navigation \&\\inspection};
\node[grp, right=10mm of e2] (e3) {Channel\\ablation};
\node[grp, right=10mm of e3] (e4) {Statistical\\analysis};
\draw[arr] (e1) -- (e2);
\draw[arr] (e2) -- (e3);
\draw[arr] (e3) -- (e4);
\draw[darr] (lqg.south) -- ++(0,-6mm) -| (e1.north);
\end{tikzpicture}
\caption{Methodology workflow of the flapping-wing sensing and navigation study.}
\label{fig:workflow}
\end{figure*}

\subsection{Simulation Framework and Vehicle Model}
\label{sec:method_vehicle}

The vehicle is modelled in the MuJoCo rigid-body physics engine, which integrates the six-degree-of-freedom body dynamics, while all aerodynamic forces are supplied externally by the blade-element model of Section~\ref{sec:method_aero}, since the engine does not itself resolve the unsteady low-Reynolds-number flow at this scale. The wings are represented as prescribed-motion, massless aerodynamic surfaces whose kinematics are imposed at each timestep; this choice follows established practice for quasi-steady flapping simulation and avoids a spurious inertial-reaction artefact that arises when finite-mass wings are driven on a free body, while preserving the forces and moments that the wings transmit to the body. Each wing follows a sinusoidal stroke angle $\phi_s(t) = \Phi\cos(2\pi f t)$, with the feathering angle switching smoothly between $\pm45^\circ$ at each stroke reversal; the hover stroke amplitude, $\Phi = 72.6^\circ$, is the value at which the cycle-averaged lift equals the weight. The thrust, roll, and pitch commands act as common-mode amplitude, differential amplitude, and stroke-offset changes respectively, and yaw is commanded through a split-cycle asymmetry. The body frame is defined with its $x$ axis forward, $y$ axis to the left, and $z$ axis upward, with origin at the centre of mass. 

Figure~\ref{fig:model} shows the modelled vehicle and the discretization of its wings. Panel (a) gives the body, the two wings, and the body coordinate frame, with the principal dimensions marked, and panel (b) shows one wing divided into the $N_s = 20$ spanwise blade-element strips used for the aerodynamic computation of Section~\ref{sec:method_aero}, together with the stroke kinematics imposed in hover.

\begin{figure*}[t]
\centering
\includegraphics[width=0.62\linewidth]{figures/model_iso.png}\hfill
\includegraphics[width=0.36\linewidth]{figures/model_wing_detail.png}
\caption{Simulated vehicle geometry and blade-element wing discretization.}
\label{fig:model}
\end{figure*}

The principal physical parameters of the modelled vehicle are collected in Table~\ref{tab:vehicle}, and are used consistently throughout the aerodynamic and control models that follow.

\begin{table}[t]
\centering
\caption{Physical and operating parameters of the modelled vehicle.}
\label{tab:vehicle}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lll@{}}
\toprule
Parameter & Symbol & Value \\
\midrule
Total mass & $m$ & $77.4$~mg \\
Weight & $mg$ & $759.3~\mu$N \\
Wing length (root to tip) & $R$ & $12.6$~mm \\
Tip-to-tip span & $2R$ & $25.2$~mm \\
Maximum chord & $c_{\max}$ & $3.5$~mm \\
Strips per wing & $N_s$ & $20$ \\
Roll inertia & $I_{xx}$ & $1.71\times10^{-10}$~kg\,m$^2$ \\
Pitch inertia & $I_{yy}$ & $6.04\times10^{-10}$~kg\,m$^2$ \\
Yaw inertia & $I_{zz}$ & $7.31\times10^{-10}$~kg\,m$^2$ \\
Hover flapping frequency & $f$ & $80$~Hz \\
Cruise speed & $V_c$ & $0.05$~m\,s$^{-1}$ \\
Physics timestep & & $1\times10^{-4}$~s \\
Nominal control rate & & $1$~kHz \\
Wingtip-strike radius & & $13.3$~mm \\
\bottomrule
\end{tabular*}
\end{table}

\subsection{Rigid-Body Dynamics and Linearized Model}
\label{sec:method_dynamics}

The vehicle is treated as a rigid body of mass $m$ with diagonal inertia tensor $\mathbf{I} = \mathrm{diag}(I_{xx}, I_{yy}, I_{zz})$. Writing $\mathbf{p}$ for world-frame position, $\mathbf{v}$ for world-frame velocity, $\boldsymbol{\eta} = [\phi, \theta, \psi]^\top$ for the Euler angles, and $\boldsymbol{\omega}$ for the body angular velocity, the equations of motion are
\begin{equation}
    \dot{\mathbf{p}} = \mathbf{v}, \qquad m\,\dot{\mathbf{v}} = \mathbf{R}(\boldsymbol{\eta})\,\mathbf{F}_b - m g\,\mathbf{e}_3,
    \label{eq:trans}
\end{equation}
\begin{equation}
    \dot{\boldsymbol{\eta}} = \mathbf{W}(\boldsymbol{\eta})\,\boldsymbol{\omega}, \qquad \mathbf{I}\,\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathbf{I}\,\boldsymbol{\omega} = \boldsymbol{\tau}_b,
    \label{eq:rot}
\end{equation}
where $\mathbf{R}$ is the body-to-world rotation, $\mathbf{W}$ maps body rates to Euler-angle rates, $\mathbf{e}_3 = [0,0,1]^\top$, and $\mathbf{F}_b$, $\boldsymbol{\tau}_b$ are the aerodynamic force and torque supplied by the blade-element model. For control design, the dynamics are linearized about the nominal hovering condition, giving
\begin{equation}
    \dot{\mathbf{x}}(t) = \mathbf{A}\,\mathbf{x}(t) + \mathbf{B}\,\mathbf{u}(t) + \mathbf{E}\,w(t),
    \label{eq:cont_ss}
\end{equation}
where $\mathbf{x} = [v_x, v_y, v_z, \omega_x, \omega_y, \phi, \theta, z_I]^\top$ collects the velocity, roll-rate and pitch-rate, and roll and pitch perturbations together with an integrated-altitude term $z_I$, $\mathbf{u} = [u_T, u_\phi, u_\theta]^\top$ holds the thrust, roll, and pitch commands, and $w(t)$ is a roll-axis disturbance developed in Section~\ref{sec:method_control}. The matrices $\mathbf{A}$ and $\mathbf{B}$ are obtained numerically, by central finite differences of the cycle-averaged aerodynamic wrench with respect to the body velocities, the body rates, and each command about the hover trim. Yaw and yaw rate are excluded from this model and are regulated by a separate heading loop. Because control is applied at a fixed interval $T_c$, the model is discretized exactly under a zero-order hold through a single matrix exponential of the augmented generator,
\begin{equation}
    \begin{bmatrix} \mathbf{A}_d & \mathbf{B}_d \\ \mathbf{0} & \mathbf{I} \end{bmatrix} = \exp\!\left( \begin{bmatrix} \mathbf{A} & \mathbf{B} \\ \mathbf{0} & \mathbf{0} \end{bmatrix} T_c \right),
    \label{eq:zoh}
\end{equation}
so that $\mathbf{A}_d$ and $\mathbf{B}_d$ are exact for the given $T_c$ rather than first-order approximations, which matters when the control interval is lengthened in the robustness study.

\subsection{Blade-Element Aerodynamic Model}
\label{sec:method_aero}

The forces produced by each wing are computed with a quasi-steady blade-element model, in which the wing is divided into $N_s$ spanwise strips and the force on each strip is obtained from its local velocity and orientation. The strip at radial station $r$ has chord $c(r)$ and moves, by virtue of the prescribed flapping, with local speed $U_t(r,t) = r\,\dot{\phi}_s(t)$. Three force contributions are modelled: a translational (circulatory) term, which dominates mid-stroke,
\begin{equation}
    \mathrm{d}\mathbf{F}_{\mathrm{trans}} = \tfrac{1}{2}\,\rho\, c(r)\, U_t^2\, \big[ C_L(\alpha)\,\hat{\mathbf{e}}_L + C_D(\alpha)\,\hat{\mathbf{e}}_D \big]\,\mathrm{d}r,
    \label{eq:Ftrans}
\end{equation}
with $C_L(\alpha) = C_{L,\max}\sin 2\alpha$ and $C_D(\alpha) = C_{D,0} + C_{D,\max}\sin^2\alpha$; a rotational term accounting for force generation at stroke reversal,
\begin{equation}
    \mathrm{d}\mathbf{F}_{\mathrm{rot}} = \rho\, C_{\mathrm{rot}}\, c^2(r)\, U_t\, \dot{\alpha}\, \hat{\mathbf{n}}\,\mathrm{d}r, \quad C_{\mathrm{rot}} = \pi\!\left( \tfrac{3}{4} - \hat{x}_0 \right);
    \label{eq:Frot}
\end{equation}
and an added-mass term reflecting the inertia of the displaced fluid,
\begin{equation}
    \mathrm{d}\mathbf{F}_{\mathrm{am}} = -\rho\,\tfrac{\pi}{4}\, c^2(r)\, a_n\, \hat{\mathbf{n}}\,\mathrm{d}r.
    \label{eq:Fam}
\end{equation}
The net force and torque transmitted by one wing are the strip sums $\mathbf{F}_w = \sum_i \mathrm{d}\mathbf{F}_i$ and $\boldsymbol{\tau}_w = \sum_i (\mathbf{r}_i - \mathbf{r}_{\mathrm{cm}}) \times \mathrm{d}\mathbf{F}_i$, and the total wrench is the sum over both wings. The empirical coefficients follow values reported for revolving insect wings, with $C_{L,\max}=1.8$, $C_{D,0}=0.2$, $C_{D,\max}=1.8$, and $\hat{x}_0=0.25$. In free-air hover the two wings are symmetric and produce no net roll torque; it is the breaking of this symmetry by a nearby wall, described next, that produces the sensing signal.

\subsection{Near-Wall Interaction Law}
\label{sec:method_wingwash}

When a wing operates near a surface, its lift is modified relative to the free-air value, an effect that is distance-dependent and well established for flapping-wing flight \cite{cheeseman1955, maeda2013, lua2016}. Adopting the classical ground-effect relation at the level of the individual strip, the lift of a strip at perpendicular distance $d$ from a wall is scaled by
\begin{equation}
    \kappa(d) = \min\!\left( 1 + K_{\mathrm{GE}} \left( \frac{R}{4d} \right)^2,\; \kappa_{\max} \right),
    \label{eq:kappa}
\end{equation}
with $K_{\mathrm{GE}} = 1.0$ and $\kappa_{\max} = 2.0$ throughout, where $\kappa_{\max}$ caps the enhancement at close range to prevent the non-physical divergence of the expression as $d\to 0$. It is stated plainly that Eq.~\eqref{eq:kappa} is imposed rather than derived: the simulation does not compute the near-wall flow but applies this scalar multiplier to the modelled strip lift, so $K_{\mathrm{GE}}$ is the single parameter on which the quantitative characterization depends, and its influence is examined explicitly. When the vehicle sits beside a wall, the strips of the near wing lie at smaller distances than those of the far wing and are enhanced more strongly, and the resulting left-right asymmetry produces a net body-axis roll torque,
\begin{equation}
    \tau_x^{\mathrm{wall}} \approx \sum_{i} \big( \kappa(d_i) - 1 \big)\, \mathrm{d}F_{z,i}\, y_i,
    \label{eq:tauwall}
\end{equation}
written here as the far-field, vertical-lift reduction of the exact moment that the implementation forms as a cross-product sum over strips. The sign of $\tau_x^{\mathrm{wall}}$ indicates which side the wall lies on, and its magnitude varies with distance, so the roll torque is a distance-dependent, side-resolving proximity signal produced by the vehicle's own propulsion. This torque appears to the controller as an external disturbance and is recovered by the observer of Section~\ref{sec:method_control}, which is what makes it available for navigation. This study refers to it as the wing-wash signal. Figure~\ref{fig:sensing} illustrates this geometry. The near wing, lying at smaller strip distances $d_i$, is enhanced more strongly than the far wing, and the resulting left-right lift asymmetry produces the roll torque $\tau_x^{\mathrm{wall}}$ that rolls the vehicle away from the wall; the same panel shows the seven forward-facing feeler rays that provide the complementary tactile cue.

\begin{figure}[t]
\centering
\includegraphics[width=\linewidth]{figures/model_sensing_geometry.png}
\caption{Wing-wash asymmetry and forward feeler-ray sensing geometry.}
\label{fig:sensing}
\end{figure}

\subsection{Control and Disturbance Estimation}
\label{sec:method_control}

Stabilized flight uses a linear-quadratic-Gaussian (LQG) regulator acting on a Kalman-filter state estimate. The regulator gain $\mathbf{K}$ minimizes the infinite-horizon discrete cost $J = \sum_{k} ( \mathbf{x}_k^\top \mathbf{Q}\,\mathbf{x}_k + \mathbf{u}_k^\top \mathbf{R}_u\,\mathbf{u}_k )$, obtained from the discrete algebraic Riccati equation, with the diagonal weights $\mathbf{Q}$ and $\mathbf{R}_u$ chosen to produce a stable hovering operating point. The element on which the sensing idea depends is the disturbance observer. The estimator state is augmented with a roll-axis disturbance $\delta$, modelled as slowly varying,
\begin{equation}
    \begin{bmatrix} \mathbf{x}_{k+1} \\ \delta_{k+1} \end{bmatrix} = \begin{bmatrix} \mathbf{A}_d & \mathbf{E}_d \\ \mathbf{0} & 1 \end{bmatrix} \begin{bmatrix} \mathbf{x}_k \\ \delta_k \end{bmatrix} + \begin{bmatrix} \mathbf{B}_d \\ \mathbf{0} \end{bmatrix} \mathbf{u}_k + \begin{bmatrix} \mathbf{0} \\ 1 \end{bmatrix} w^{\delta}_k,
    \label{eq:aug}
\end{equation}
and the augmented Kalman filter returns, each step, an estimate $\hat{\delta}_k$ alongside the state estimate $\hat{\mathbf{x}}_k$. The control law regulates the estimated state while feeding the estimated disturbance forward for rejection, $\mathbf{u}_k = -\mathbf{K}\,\hat{\mathbf{x}}_k - \mathbf{K}_{\delta}\,\hat{\delta}_k + \mathbf{u}_{\mathrm{ref},k}$, where $\mathbf{u}_{\mathrm{ref},k}$ is the reference from the navigation layer. The key point, in the manner of disturbance-observer-based methods \cite{chen2016disturbance}, is that $\hat{\delta}_k$ is not merely a quantity to reject: when a wall is present it estimates the wall-induced torque of Eq.~\eqref{eq:tauwall}, so the same estimator that stabilizes the vehicle also measures its lateral surroundings. No component of this architecture is learned; all gains follow analytically from the linearized model. The deployed weights, which differ from the source-module defaults, are reported in Table~\ref{tab:params} so that the controller is reproducible.

Figure~\ref{fig:hover} illustrates the stabilizing action of this regulator in free air. Released with a $10^\circ$ attitude offset in both pitch and roll, the uncontrolled vehicle diverges past $45^\circ$ within about $124$~ms, whereas the regulated vehicle settles to hover and rejects a roll gust and a pitch gust applied at $400$ and $800$~ms while holding its height within about $1$~mm of the $50$~mm reference.

\begin{figure}[t]
\centering
\includegraphics[width=\linewidth]{figures/fig03_hover_control.png}
\caption{Open-loop divergence and closed-loop LQG hover with gust rejection.}
\label{fig:hover}
\end{figure}

\subsection{Sensor Suite and Noise Model}
\label{sec:method_noise}

The vehicle carries two sensory channels. The first is the aerodynamic proximity channel, which is not a physical sensor but the roll-axis disturbance estimate of Section~\ref{sec:method_control}, and therefore adds no mass or power. The second is a set of seven forward-facing feelers, modelled as rays cast from the nose at $0^\circ$, $\pm30^\circ$, $\pm50^\circ$, and $\pm90^\circ$ to the heading, each returning the distance to the first wall it meets up to a maximum range of $0.16$~m. It is stated at the outset, and revisited in the ablation, that these feelers are idealized: they return exact geometric range through free space, rather than the contact or bending signals of a physical tactile antenna. All measurements are corrupted by an additive Gaussian noise model,
\begin{equation}
    \tilde{y}_{j,k} = y_{j,k} + \nu_{j,k}, \qquad \nu_{j,k} \sim \mathcal{N}\!\left(0, (\lambda\,\sigma_j)^2\right),
    \label{eq:noise}
\end{equation}
where $\sigma_j$ is the baseline standard deviation of channel $j$, listed in Table~\ref{tab:noise}, and $\lambda$ is a multiplier, unity at the nominal condition and swept to probe robustness. This noise model is an assumption of the study rather than a hardware measurement, and the aerodynamic estimate is never noised directly; noise reaches it only through the inertial measurements and the estimator, which is the physically faithful path.

\begin{table}[t]
\centering
\caption{Baseline standard deviations of the sensor-noise model at $\lambda = 1$.}
\label{tab:noise}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lll@{}}
\toprule
Measurement channel & $\sigma_j$ & Unit \\
\midrule
Feeler range & $0.002$ & m \\
Attitude (pitch, roll) & $0.005$ & rad \\
Angular rate & $0.020$ & rad\,s$^{-1}$ \\
Translational velocity & $0.005$ & m\,s$^{-1}$ \\
Heading & $0.005$ & rad \\
Height & $0.001$ & m \\
\bottomrule
\end{tabular*}
\end{table}

\subsection{Reactive Navigation Layer}
\label{sec:method_nav}

The navigation layer maps instantaneous sensory inputs to attitude and velocity references through fixed reactive laws, with no map, planner, or learned policy, and is built so that either channel may be disabled independently for the ablation. Lateral centring drives the low-pass-filtered wing-wash estimate toward zero while damping lateral velocity, subject to a small roll saturation. Forward speed is reduced as the forward feeler approaches an obstacle, and heading is steered toward the more open of the two forward diagonals, so corridors are followed and corners are turned. When the forward path is blocked and a side is open, the geometry is a junction and the vehicle turns into it; when the forward path is clear and exactly one side reads open, it is an opening in an intact wall, and the vehicle holds its line under a feeler-vetoed law that follows the remaining wall while logging the opening. This arbitration between an opening to inspect and a passage to follow is the mechanism on which the inspection task rests.

The fixed values of these reactive laws, together with the regulator and estimator weights of Section~\ref{sec:method_control}, are collected in Table~\ref{tab:params}. They are the values used in every closed-loop flight reported here, and none is changed between sensing configurations.

\begin{table*}[t]
\centering
\begin{threeparttable}
\caption{Regulator, estimator, and navigation parameters as deployed in all closed-loop flights.}
\label{tab:params}
\footnotesize\setlength{\tabcolsep}{3.5pt}
\begin{tabular*}{\textwidth}{@{\extracolsep{\fill}}lll@{}}
\toprule
Parameter & Value & Applies to \\
\midrule
\multicolumn{3}{@{}l}{\emph{LQG regulator}} \\
State weights $\mathbf{Q}$ (diagonal)\tnote{a} & $150,\ 150,\ 20,\ 2,\ 2,\ 250,\ 250,\ 6\times10^{4}$ & $v_x, v_y, v_z, \omega_x, \omega_y, \phi, \theta$, $z_I$ \\
Input weights $\mathbf{R}_u$ (diagonal) & $50,\ 50,\ 50$ & thrust, roll, pitch commands \\
\midrule
\multicolumn{3}{@{}l}{\emph{Kalman filter and disturbance observer}} \\
Process-noise weights (diagonal) & $1,\ 1,\ 0.05,\ 30,\ 30,\ 0.02,\ 0.02,\ 10^{-6}$ & same state ordering as $\mathbf{Q}$ \\
Disturbance process-noise weight & $2\times10^{8}$ & roll-axis disturbance $\delta$ \\
Measurement-noise weights (diagonal) & $10^{-3},\ 100,\ 100,\ 10^{-4},\ 10^{-4},\ 10^{-7}$ & $v_z, \omega_x, \omega_y, \phi, \theta$, altitude \\
Wing-wash low-pass time constant & $0.10$~s & $\hat{\delta}$ before the centring law \\
\midrule
\multicolumn{3}{@{}l}{\emph{Reactive navigation}} \\
Centring gain & $1.5\times10^{-5}$~s$^{2}$ & roll reference per unit filtered $\hat{\delta}$ \\
Lateral-velocity damping & $3.0$~rad\,s\,m$^{-1}$ & roll reference \\
Roll-reference saturation & $\pm 2.5^\circ$ & cruise ($\pm 2^\circ$ while turning) \\
Steering gain & $5.0$~rad\,s$^{-1}$\,m$^{-1}$ & difference of the $\pm30^\circ$ feelers, each clipped at $0.11$~m \\
Stop and clear distances & $0.042$~m, $0.090$~m & forward feeler: slow to a stop, resume cruise \\
Safety buffer and veer rate & $0.020$~m, $0.9$~rad\,s$^{-1}$ & side and $\pm50^\circ$ feelers \\
Turn yaw rate & $0.5$~rad\,s$^{-1}$ & pivot at a blocked corner \\
Opening threshold & $0.10$~m & side-feeler range above which a wall reads open \\
Wall-follow gain and standoff & $2.0$~rad\,m$^{-1}$, $0.030$~m & feeler-vetoed law beside an opening \\
Cruise height & $0.05$~m & altitude reference \\
Detection match tolerance & $60$~mm & logged to true opening midpoint \\
\bottomrule
\end{tabular*}
\begin{tablenotes}\footnotesize
\item[a] In navigation flights the regulator's feedback on the horizontal velocities $v_x$ and $v_y$ is set to zero, and those velocities are commanded by the navigation layer instead.
\end{tablenotes}
\end{threeparttable}
\end{table*}

\subsection{Experimental Design}
\label{sec:method_experiments}

The evaluation is organized as the sequence of experiment groups shown in Figure~\ref{fig:workflow} and summarized in Table~\ref{tab:experiments}. The first group characterizes the wing-wash signal as an instrument in open loop, with the vehicle held at prescribed poses, so its range, resolution, observer fidelity, and two-wall response are measured in isolation from closed-loop dynamics. The second evaluates reactive navigation and opening inspection, first on crafted courses and then across a randomized ensemble of passages. Each randomized course is generated by independently sampling the number of legs, the turn directions, the corridor widths, and the number, side, position, and length of the wall openings, subject to constraints that keep the geometry realizable, and each course is flown once, so the ensemble measures performance across geometries rather than repeatability on one. The third group ablates each sensing channel in turn on geometrically identical courses, using the same generator and flight seeds across configurations, so the comparison isolates the contribution of a channel rather than the effect of a re-tuned controller; no gain is retuned for any configuration. The fourth group applies the statistical analysis described below.

\begin{table}[t]
\centering
\caption{Numerical experiment groups and the quantities each measures.}
\label{tab:experiments}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabularx}{\columnwidth}{@{}L{2.4cm}Y@{}}
\toprule
Group & Principal quantities \\
\midrule
Signal characterization & Range, resolution, observer gain, two-wall superposition \\
Navigation \& inspection & Completion, strike-free completion, detection, localization \\
Channel ablation & Attribution across full, no-wing-wash, no-feelers, open-loop \\
Statistical analysis & Paired differences, confidence intervals, equivalence \\
\bottomrule
\end{tabularx}
\end{table}

The sampling distributions of the course generator are listed in Table~\ref{tab:generator}, for the primary ensemble and for the leg-sweep ensemble used in Section~\ref{sec:results_regimes}; the two differ only in leg count and corridor width.

\begin{table}[t]
\centering
\caption{Sampling distributions of the randomized-course generator.}
\label{tab:generator}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabularx}{\columnwidth}{@{}lYY@{}}
\toprule
Quantity & Primary ensemble & Leg sweep \\
\midrule
Courses & $40$ & $100$ ($20$ per leg count) \\
Number of legs & $2$ or $3$, equal probability & $2, 3, 4, 5, 6$ (fixed per cell) \\
Corridor width, per leg & uniform, $64$ to $76$~mm & uniform, $58$ to $64$~mm \\
Leg length & \multicolumn{2}{l}{uniform, $160$ to $260$~mm} \\
Turn direction & \multicolumn{2}{l}{left or right, equal probability} \\
Openings per leg & \multicolumn{2}{l}{$0$ (probability $0.4$) or $1$ ($0.6$)} \\
Opening side & \multicolumn{2}{l}{left or right, equal probability} \\
Opening length & \multicolumn{2}{l}{uniform, $15$ to $90$~mm} \\
Opening centre & \multicolumn{2}{l}{uniform, $0.40$ to $0.62$ of leg length} \\
Opening retained if & \multicolumn{2}{l}{span within $0.25$ to $0.82$ of leg length} \\
Course accepted if & \multicolumn{2}{l}{non-adjacent legs apart; $\geq 1$ opening} \\
\bottomrule
\end{tabularx}
\end{table}

Figure~\ref{fig:courses} shows a sample of the generated primary-ensemble courses, illustrating the range of leg counts, turn directions, and wall-opening placements evaluated; each course is flown once, so the ensemble measures performance across geometries rather than repeatability on any one.

\begin{figure}[t]
\centering
\includegraphics[width=\linewidth]{figures/design_a_courses.png}
\caption{Representative randomized courses from the primary ensemble.}
\label{fig:courses}
\end{figure}

\subsection{Metrics and Statistical Analysis}
\label{sec:method_stats}

Two outcome quantities are kept distinct throughout. A run is a \emph{completion} if the vehicle reaches the goal; a \emph{wingtip strike} is registered whenever the centre-of-mass-to-wall distance falls below the wingtip-strike radius; and a run is \emph{strike-free} only if it completes with no strike. For inspection, a logged detection is matched to a ground-truth opening when it lies on the same wall and within a fixed tolerance of the true midpoint, and the per-passage and per-opening detection rates are reported in both their conditional form, over completed courses, and their end-to-end form, over all generated courses. The central comparison, between the full configuration and the configuration with the aerodynamic channel removed, is analysed under a pre-registered paired design: for each course the two configurations are flown under matched noise seeds, and the paired difference in strike-free completion is reported with a 95\% confidence interval obtained by cluster bootstrap over courses, together with an exact McNemar test and a pre-registered equivalence margin fixed before the results were examined. This design, and the correction of a simulation coupling that initially confounded it, are the methodological core of the study, and the analysis was conducted with the open-source scientific Python stack.

\subsection{Reproducibility and Ethics}
\label{sec:method_repro}

The study involves no human or animal subjects, so no institutional review-board approval or participant consent was required. To support independent verification, the complete simulation framework, controller, characterization experiments, ablation study, and randomized-course benchmark are released as open-source software, with each reported result persisted to file and reproducible from a frozen repository state.

\section{Results}
\label{sec:results}

This section reports the measured outcomes of the experiment groups described in Section~\ref{sec:method_experiments}, in the order of characterization, reactive inspection, and channel ablation. Findings are stated objectively; their interpretation is deferred to Section~\ref{sec:discussion}. Unless otherwise noted, all results use the nominal ground-effect coefficient $K_{\mathrm{GE}} = 1.0$ and nominal sensor noise $\lambda = 1$, and all closed-loop results are from the corrected simulation harness described in Section~\ref{sec:method_stats}.

\subsection{Characterization of the Wing-Wash Signal}
\label{sec:results_sensor}

The wall-induced roll disturbance of Eq.~\eqref{eq:tauwall}, as recovered by the observer, was characterized in open loop with the vehicle held at prescribed poses. Four properties are reported. First, the static response: sweeping a single wall from $2$ to $40$~mm, the cycle-averaged induced roll acceleration rises to a saturation knee below about $6$~mm, where the near-wing enhancement reaches the cap, and decays monotonically beyond it. The saturated region lies entirely within the $13.3$~mm wingtip-strike radius, so the usable band runs from roughly $8$ to $40$~mm. The decay within this band is not a single power law; power-law fits over sub-intervals are given in Table~\ref{tab:falloff}. A fit over the full band gives an exponent of $-3.59$, whereas the well-conditioned far-field fit over $20$ to $40$~mm gives $-3.63$ with $R^2 = 0.998$. Figure~\ref{fig:signal} shows this static response. Panel (a) gives the wall-induced roll torque against wall distance, with the saturation knee near $6$~mm, and panel (b) gives the corresponding roll acceleration on logarithmic axes together with the far-field fit and the observer noise floor, above which the signal remains across the whole swept range.

\begin{figure*}[t]
\centering
\includegraphics[width=0.9\linewidth]{figures/fig04_wingwash_signal.png}
\caption{Static response of the wing-wash signal to a single wall: (a) roll torque and (b) roll acceleration with far-field fit and noise floor.}
\label{fig:signal}
\end{figure*}

\begin{table}[t]
\centering
\caption{Power-law fits of the roll-acceleration decay over distance sub-intervals.}
\label{tab:falloff}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}llll@{}}
\toprule
Regime & Range (mm) & Exponent & $R^2$ \\
\midrule
Full band & 8 to 40 & $-3.59$ & $0.974$ \\
Past knee & 10 to 40 & $-3.90$ & $0.990$ \\
Near shoulder & 10 to 18 & $-3.16$ & $0.914$ \\
Far field & 20 to 40 & $-3.63$ & $0.998$ \\
\bottomrule
\end{tabular*}
\end{table}

Second, the resolution. In noise-free hover away from any wall, the filtered estimate fluctuates with a standard deviation of $24.7$~rad\,s$^{-2}$ about a mean indistinguishable from zero. Dividing this noise floor by the local slope of the static response gives the distance resolution, which is below one millimetre across the usable band and reaches approximately $7$ to $13~\mu$m in the $10$ to $16$~mm region where the response is steepest. This figure is a noise-equivalent distance under the assumed noise model, not a hardware resolution. Third, the observer fidelity. Comparing the settled estimate with the true induced acceleration, the observer recovers the signal with a direct-current gain of approximately $0.57$, essentially invariant with distance over the $15$ to $30$~mm range; below about $15$~mm the measurement is unreliable owing to position-hold drift. A step change in wall distance produces a ten-to-ninety-percent rise in about $13$~ms. Fourth, the two-wall response. Between two walls the induced roll acceleration is odd-symmetric in lateral offset and nulls at the corridor centreline; at offsets of $4$, $8$, and $12$~mm the directly measured two-wall response equals the difference of the single-wall response to a ratio of unity to fifteen decimal places. Figure~\ref{fig:observer_twowall} illustrates the last two properties. Panel (a) shows the observer gain against wall distance, which stays between $0.53$ and $0.59$ over the $15$ to $30$~mm band, and panel (b) shows the odd-symmetric two-wall response, which crosses zero at the centreline with a slope of about $109$~rad\,s$^{-2}$ per millimetre and coincides with the single-wall superposition.

\begin{figure*}[t]
\centering
\begin{subfigure}[b]{0.48\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/fig06_observer_gain.png}
\caption{}
\end{subfigure}\hfill
\begin{subfigure}[b]{0.48\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/fig05_two_wall_centering.png}
\caption{}
\end{subfigure}
\caption{Observer fidelity and two-wall response: (a) ratio of estimated to true roll disturbance and (b) net roll acceleration against lateral offset in a corridor.}
\label{fig:observer_twowall}
\end{figure*}

\subsection{Detection Floor, Localization, and Sizing}
\label{sec:results_detection}

The elementary detection limit was measured by placing a single opening of controlled length in one wall of a $64$~mm corridor and sweeping its length, with five noise realizations at each length under nominal sensor noise. The outcomes are given in Table~\ref{tab:detection}. The opening was detected in all five realizations at every length from $12$ to $60$~mm and in none at $10$~mm, so the detection floor lies between $10$ and $12$~mm. Mean localization error of the estimated opening midpoint was below one millimetre for openings up to $35$~mm, with a largest single-flight error of $1.7$~mm, and rose to between $2$ and $4$~mm for openings of $40$ to $60$~mm. The raw length estimate is biased short, and a single linear calibration fitted to the fully detected lengths (slope $0.884$, offset $-7.98$~mm) recovered the true length with a per-flight root-mean-square residual of $0.98$~mm over $12$ to $40$~mm and $1.05$~mm over $12$ to $60$~mm; because the calibration is fitted to these same flights, the residuals are in-sample. The $80$~mm opening marks the upper limit of the procedure: three of the five flights there did not complete the corridor, one of them logging no opening and two logging a truncated one, while the two flights that did complete sized the opening to within $1.4$~mm.

\begin{table}[t]
\centering
\caption{Detection rate, localization error, and calibrated size error versus opening length.}
\label{tab:detection}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{threeparttable}
\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}llll@{}}
\toprule
True length (mm) & Detection & Loc. error (mm) & Size error (mm) \\
\midrule
10 & 0/5 & n/a & n/a \\
12 & 5/5 & 0.72 & $-0.98$ \\
15 & 5/5 & 0.59 & $-0.04$ \\
18 & 5/5 & 0.49 & $-0.66$ \\
20 & 5/5 & 0.36 & $+0.30$ \\
25 & 5/5 & 0.61 & $+0.47$ \\
30 & 5/5 & 0.58 & $+0.54$ \\
35 & 5/5 & 0.96 & $+1.75$ \\
40 & 5/5 & 2.03 & $-0.05$ \\
50 & 5/5 & 3.83 & $-1.10$ \\
60 & 5/5 & 3.74 & $-0.24$ \\
80\tnote{a} & 4/5 & 7.29 & $-19.0$ \\
\bottomrule
\end{tabular*}
\begin{tablenotes}\footnotesize
\item[a] Three of the five flights at $80$~mm did not complete the corridor, and this length is excluded from the calibration fit; the two completed flights give errors of $2.8$ and $4.1$~mm in location and $-1.3$ and $-1.5$~mm in size.
\end{tablenotes}
\end{threeparttable}
\end{table}

\subsection{Randomized-Course Inspection}
\label{sec:results_batch}

Before the randomized ensemble, the integrated system was flown on a crafted three-leg course containing three wall openings, and Figure~\ref{fig:mission} shows the resulting flight. The vehicle centred itself in each leg, negotiated both corners, and reached the goal in $17.2$~s with a minimum clearance of $21.6$~mm, logging all three openings on the correct wall.

\begin{figure}[t]
\centering
\includegraphics[width=\linewidth]{figures/fig09_example_mission.png}
\caption{Example flight on a crafted three-leg course with true and logged wall openings.}
\label{fig:mission}
\end{figure}

The integrated system was then evaluated across a primary ensemble of $40$ independently generated multi-turn courses, each flown once under nominal noise, with the aerodynamic and tactile channels both active. The vehicle reached the goal on $35$ of the $40$ courses. Of these, five reached the goal having incurred a wall strike along the way, so the strike-free completion rate was $30$ of $40$. Of the openings present, $43$ were detected out of the $58$ in completed courses, which is $74$ percent conditionally and $43$ of $68$, or $63$ percent, over all generated courses. The mean localization error of detected openings was $7.9$~mm, with a median of $6.9$~mm, and the system produced one false positive across the $35$ completed courses. These figures are summarized in Table~\ref{tab:batch}.

\begin{table}[t]
\centering
\caption{Randomized-course inspection outcomes on the primary ensemble.}
\label{tab:batch}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lll@{}}
\toprule
Metric & Conditional & End-to-end \\
\midrule
Courses generated & -- & 40 \\
Reached goal & -- & 35/40 (88\%) \\
Reached goal, strike-free & -- & 30/40 (75\%) \\
Per-passage detection & 30/35 (86\%) & 30/40 (75\%) \\
Per-opening detection & 43/58 (74\%) & 43/68 (63\%) \\
Localization error (mm) & $7.9$ (mean), $6.9$ (median) & -- \\
False positives per course & 0.03 (1 in 35) & 0.025 (1 in 40) \\
\bottomrule
\end{tabular*}
\end{table}

\subsection{Channel Ablation}
\label{sec:results_ablation}

The contribution of each sensing channel was measured by removing it on the identical primary ensemble, with all other factors held fixed. Table~\ref{tab:ablation} reports goal-reaching and detection for the four configurations, and Figure~\ref{fig:abl_bars} shows them. Removing the feelers reduced goal-reaching from $35$ to $16$ of $40$ and produced no logged detections, since disabling the feelers also disables the opening logger. Removing all environmental sensing (open-loop) produced no goal-reaching. Removing the aerodynamic channel while retaining the feelers produced $32$ of $40$ goal-reaching, against $35$ for the full configuration, with detection of a comparable number of openings.

\begin{table}[t]
\centering
\begin{threeparttable}
\caption{Ablation outcomes across the four sensing configurations.}
\label{tab:ablation}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lllll@{}}
\toprule
Configuration & Reached & Strike-free & Per-opening & FP/course \\
\midrule
Full & 35/40 & 30/40 & 43/58 & 0.03 \\
No wing-wash & 32/40 & 29/40 & 41/54 & 0.03 \\
No feelers & 16/40 & 15/40 & 0/25\tnote{a} & 0.00 \\
Open-loop & 0/40 & 0/40 & -- & -- \\
\bottomrule
\end{tabular*}
\begin{tablenotes}\footnotesize
\item[a] The zero detection without feelers is structural, since the opening logger is disabled with the feeler channel, and is not an observed detection miss.
\end{tablenotes}
\end{threeparttable}
\end{table}

\begin{figure}[t]
    \centering
    \includegraphics[width=\linewidth]{figures/fig10_ablation_truewall.png}
    \caption{Goal-reaching, strike-free completion, and end-to-end opening detection for the four sensing configurations.}
    \label{fig:abl_bars}
\end{figure}

The clearance statistics of the full and no-wing-wash configurations are compared in Figure~\ref{fig:abl_clear}, which shows the per-flight minimum clearance over the paired-seed flights of Section~\ref{sec:results_stats} against the wingtip-strike radius. On the primary ensemble, the mean minimum clearance was $18.3$~mm for the full configuration and $17.4$~mm without the aerodynamic channel, the worst single clearance was $2.2$~mm and $1.6$~mm respectively, and both configurations incurred wall strikes on a similar number of courses.

\begin{figure}[t]
    \centering
    \includegraphics[width=\linewidth]{figures/fig13_clearance_truewall.png}
    \caption{Per-flight minimum clearance for the full and no-wing-wash configurations across the $200$ paired-seed flights.}
    \label{fig:abl_clear}
\end{figure}

\subsection{Statistical Comparison of the Aerodynamic Channel}
\label{sec:results_stats}

The central comparison, between the full configuration and the configuration with the aerodynamic channel removed, was analysed under the pre-registered paired design of Section~\ref{sec:method_stats}, using five matched noise seeds per course across the primary ensemble, for $200$ paired trials. Table~\ref{tab:stats} reports the results. The paired difference in strike-free completion was $+1.0$ percentage points in favour of the full configuration, with a $95$ percent confidence interval of $[-5.0, +6.5]$ percentage points and an exact McNemar $p$-value of $0.83$. Per-opening detection on jointly completed trials differed by $+1.1$ percentage points, and mean minimum clearance by $+0.24$~mm, both with confidence intervals spanning zero. The confidence interval on the primary metric crosses the pre-registered equivalence margin of $\pm 5$ percentage points at its upper end, so the comparison is classified as inconclusive with respect to formal equivalence at this sample size; no difference was resolved on any metric.

\begin{table}[t]
\centering
\caption{Paired comparison of the full and no-wing-wash configurations.}
\label{tab:stats}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lll@{}}
\toprule
Metric & Paired diff. & 95\% CI \\
\midrule
Strike-free completion & $+1.0$ pp & $[-5.0, +6.5]$ pp \\
Per-opening detection & $+1.1$ pp & $[-1.2, +3.5]$ pp \\
Mean min-clearance & $+0.24$ mm & $[-0.57, +0.98]$ mm \\
\bottomrule
\end{tabular*}
\end{table}

\subsection{Search for a Regime of Advantage}
\label{sec:results_regimes}

Two further analyses tested whether an operating regime exists in which the aerodynamic channel confers a measurable advantage. The first swept the number of legs from two to six, on fresh noise seeds, comparing the two configurations on strike-free completion. A difference observed in an earlier, uncorrected analysis did not reproduce: pooled over the sweep, the full configuration completed $48$ of $100$ strike-free against $42$ of $100$ without the aerodynamic channel, with an exact McNemar $p$-value of $0.29$. The second analysis degraded the forward-facing feelers by limiting their range from the nominal $0.16$~m down to $0.03$~m, comparing the two configurations at each level; the outcomes are shown in Figure~\ref{fig:feeler}. At the three widest limits the two configurations were indistinguishable. At a limit of $0.05$~m the full configuration completed $25$ of $40$ courses strike-free against $18$ of $40$ without the aerodynamic channel, a difference whose confidence interval nonetheless reaches zero and whose McNemar $p$-value is $0.118$. At a limit of $0.03$~m both configurations failed, completing zero and one of the paired courses respectively. This single, underpowered condition is the only one in which the aerodynamic channel showed an advantage, and it is not statistically established. The dropout and elevated-noise degradation axes were not evaluated. Table~\ref{tab:regimes} lists the strike-free outcome of every cell of both analyses, with the paired difference and the exact McNemar $p$-value for each.

\begin{table}[t]
\centering
\begin{threeparttable}
\caption{Strike-free completion of the full and no-wing-wash configurations in the leg-sweep and feeler-range analyses.}
\label{tab:regimes}
\footnotesize\setlength{\tabcolsep}{3pt}
\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lllll@{}}
\toprule
Condition & Full & No wing-wash & Diff. (pp) & $p$ \\
\midrule
\multicolumn{5}{@{}l}{\emph{Number of legs}} \\
2 & 13/20 & 12/20 & $+5$ & 1.000 \\
3 & 10/20 & 9/20 & $+5$ & 1.000 \\
4 & 6/20 & 7/20 & $-5$ & 1.000 \\
5 & 12/20 & 10/20 & $+10$ & 0.727 \\
6 & 7/20 & 4/20 & $+15$ & 0.375 \\
Pooled & 48/100 & 42/100 & $+6$ & 0.286 \\
\midrule
\multicolumn{5}{@{}l}{\emph{Feeler range limit}} \\
0.16 m (nominal)\tnote{a} & 32/40 & 32/40 & $0.0$ & 1.000 \\
0.12 m & 30/40 & 31/40 & $-2.5$ & 1.000 \\
0.08 m & 31/40 & 31/40 & $0.0$ & 1.000 \\
0.05 m & 25/40 & 18/40 & $+17.5$ & 0.118 \\
0.03 m\tnote{b} & 0/32 & 1/32 & $-3.1$ & 1.000 \\
\bottomrule
\end{tabular*}
\begin{tablenotes}\footnotesize
\item[a] The nominal row is the matching single-seed slice of the paired-seed runs of Section~\ref{sec:results_stats}, not a separate set of flights.
\item[b] Only $32$ of the $40$ courses were flown in both configurations at this limit.
\end{tablenotes}
\end{threeparttable}
\end{table}

\begin{figure}[t]
\centering
\includegraphics[width=\linewidth]{figures/design_c_feeler_range.png}
\caption{Strike-free completion under feeler range limitation.}
\label{fig:feeler}
\end{figure}

Figure~\ref{fig:paired} summarizes every paired comparison of the two configurations, giving the difference in strike-free completion with its $95$ percent confidence interval for the primary ensemble, the leg sweep, and each feeler-range limit, against the pre-registered $\pm 5$ percentage-point margin. No interval excludes zero, and the one point estimate that lies well outside the margin, at a feeler range of $0.05$~m, comes from a comparison that was not pre-registered and used a single seed per course.

\begin{figure*}[t]
\centering
\includegraphics[width=0.9\linewidth]{figures/fig11_paired_differences.png}
\caption{Paired differences in strike-free completion (full minus no-wing-wash) with 95\% confidence intervals across all comparisons.}
\label{fig:paired}
\end{figure*}

\subsection{Sensitivity to the Ground-Effect Coefficient}
\label{sec:results_sensitivity}

Finally, the dependence on the single uncertain modelling parameter was measured by sweeping $K_{\mathrm{GE}}$ over $0.5$ to $1.5$ times its nominal value across three control rates on a single crafted course, with five noise realizations per combination under nominal sensor noise. The resulting grid is shown in Figure~\ref{fig:kge}. At the nominal control rate of $1000$~Hz the vehicle reached the goal without a wingtip strike in all fifteen flights, with minimum clearances between $17.2$ and $20.3$~mm and no dependence on the coefficient. At the lowest coefficient every flight completed at every rate. Failures appeared only at reduced control rates and grew with the coefficient: at $500$~Hz, four of five flights completed at the nominal coefficient and one of five at $K_{\mathrm{GE}} = 1.5$, that one with a wingtip strike, while at $250$~Hz four of five completed at $K_{\mathrm{GE}} = 1.5$. Overall, $39$ of the $45$ flights reached the goal. The result is therefore insensitive to the coefficient at the nominal rate, and the margin to a stronger wall effect narrows as the control rate is lowered, non-monotonically in rate since $500$~Hz fares worse than $250$~Hz.

\begin{figure}[t]
\centering
\includegraphics[width=0.85\linewidth]{figures/design_b_kge_grid_truewall.png}
\caption{Goal-reaching across ground-effect coefficient and control rate; each cell gives the number of noise realizations, out of five, that reached the goal.}
\label{fig:kge}
\end{figure}

\section{Discussion}
\label{sec:discussion}

This section interprets the results of Section~\ref{sec:results}, relates them to prior work, and states the implications, limitations, and directions for further study. The central question posed in Section~\ref{sec:introduction} was not only whether the wall-induced aerodynamic signal can be measured, but whether it contributes to the navigation task once a tactile cue is also present. The results answer the first part affirmatively and the second part negatively, within the conditions tested, and the discussion is organized around that distinction.

\subsection{Interpretation of the Main Findings}
\label{sec:disc_interp}

Two findings stand together and must be read together. The characterization of Section~\ref{sec:results_sensor} shows that the wing-wash signal is, within the adopted model, a well-behaved proximity instrument: it has a usable range of roughly $8$ to $40$~mm, a sub-millimetre noise-equivalent distance, a stable observer gain, and a clean two-wall response. The ablation of Section~\ref{sec:results_ablation} and the paired analysis of Section~\ref{sec:results_stats} show that removing this signal from the integrated system changes performance by an amount that the experiments cannot distinguish from zero: the paired difference in strike-free completion was $+1.0$ percentage points with a confidence interval of $[-5.0, +6.5]$ and a $p$-value of $0.83$, and no difference was resolved on detection or clearance. The interval is not narrow: it is compatible with a benefit of up to about $6.5$ percentage points or a penalty of up to $5$, so the result is an absence of detected effect and not a demonstration of equivalence. Removing the tactile feelers, by contrast, reduced goal-reaching by more than half. The competence of the system is therefore attributable chiefly to the tactile channel, and the aerodynamic channel, though measurable, made no detectable contribution to the task in this regime.

This is a dissociation between what a signal \emph{is} and what it \emph{does}. The most direct explanation lies in the nature of the competing channel. The feelers modelled here are idealized: they return exact geometric range through free space, with only additive noise, and no dropout, deflection, or contact requirement. On the clean, rectilinear corridors of this study, such a channel supplies the centring and obstacle information the reactive controller needs with ample margin, leaving little for a second channel to add. The apparent redundancy is thus a property of the comparison as constructed, not a statement that the aerodynamic effect is without value in general. Supporting this reading, the no-feelers configuration, in which the aerodynamic channel operates alone, still reached the goal on $16$ of $40$ courses, well above the zero of the open-loop case; the aerodynamic signal does carry usable centring information, but it is dominated when an idealized tactile cue is also available.

The single condition in which an advantage for the aerodynamic channel appeared was the feeler-range degradation of Section~\ref{sec:results_regimes}, where limiting the feeler range to $0.05$~m produced a strike-free-completion difference of $+17.5$ percentage points in favour of the full configuration. This result is suggestive but not established: its confidence interval reaches down to zero, its $p$-value is $0.118$, it rests on a single noise seed per course, and it is one cell of a sweep that was not pre-registered. It should therefore be read as a lead for further study rather than as evidence of benefit. Its direction, however, is consistent with the explanation above, since it is precisely when the tactile channel is weakened that a second, contactless channel would be expected to matter.

\subsection{Relation to Prior Work}
\label{sec:disc_prior}

These findings connect closely to the two most relevant prior studies. Nakata et al. demonstrated that mosquitoes, and a bioinspired quadcopter, can detect surfaces from modifications of their self-induced airflow \cite{nakata2020}, establishing that aerodynamic surface sensing is physically real and useful on some platforms. The present study does not contradict this; it characterizes an analogous signal for a flapping-wing vehicle and confirms that it is measurable. What it adds is the separate question of marginal utility: whether, on a vehicle that also carries a tactile cue, the aerodynamic signal improves the task. The outcome here, within the conditions tested, is that no improvement was detected. This is also consistent with the hardware experience of Tu et al., who found that a usable aerodynamic wall signal could not be obtained on their flapping-wing platform and used wing contact instead \cite{actingIsSeeing2019}. Taken together, the prior work and the present study suggest that the existence of a near-wall aerodynamic effect does not by itself guarantee a useful navigational signal, and that whether it earns a place depends on the platform, the environment, and the alternative sensing available. The non-monotonic, saturating static response observed here is also consistent with the three-regime ground-effect behaviour reported in the flapping-wing aerodynamics literature \cite{lua2016}, which lends physical plausibility to the modelled signal even as its task utility is found to be limited.

\subsection{Implications}
\label{sec:disc_implications}

The practical implication is a cautionary one for sensor design at this scale. A sensing modality can be accurately measurable, finely resolved, and physically well motivated, and still contribute nothing to a task if a simpler channel already suffices; evaluating a system only in its complete configuration will not reveal this, and can lead to a measurable-but-redundant signal being credited with a role it does not play. The ablation methodology used here, removing each channel on identical inputs and subjecting the comparison to a pre-registered paired test, is a general safeguard against that error, and its value was demonstrated concretely when it revealed that a simulation coupling had initially confounded the comparison. The methodological implication, then, is that claims of sensory utility in such systems are best supported by ablation and by analysis that distinguishes measurability from contribution. On the positive side, the study establishes that control-only reactive navigation and opening inspection are achievable for a simulated insect-scale flapping-wing vehicle without vision, positioning, or learning, which is itself a useful baseline for confined-space operation.

\subsection{Limitations}
\label{sec:disc_limitations}

Several limitations bound these conclusions. First, the study is conducted entirely in simulation; no result has been shown to transfer to hardware, and the experience of prior investigators who could not detect the effect on a physical platform \cite{actingIsSeeing2019} underscores that this transfer is an open question. Second, the near-wall law of Eq.~\eqref{eq:kappa} is imposed rather than derived, and the sensitivity analysis of Section~\ref{sec:results_sensitivity} varies only the coefficient within that fixed functional form, so structural uncertainty in the form itself is untested. Third, the sensor-noise model is an assumption, and the sub-millimetre resolution figure is a noise-equivalent distance under that assumption rather than a demonstrated sensor resolution. Fourth, and most consequentially for the central finding, the feeler model is idealized, and the null result is conditional on that idealization; a realistically imperfect tactile channel was only partially approximated, through range limitation, and the dropout and elevated-noise degradations were not evaluated. Fifth, the statistical basis is modest: three to five noise realizations for the characterization experiments and, for the main comparison, five matched seeds across forty single-flight courses, sufficient to bound any benefit of the aerodynamic channel at roughly $6.5$ percentage points of strike-free completion, but not to certify formal equivalence within the pre-registered margin. The millimetre-level localization and sizing figures likewise come from a single straight corridor with one isolated opening; on the randomized courses the mean localization error was $7.9$~mm. Finally, the detection task concerns a geometric opening, not a specific hazard, and the correlation between an opening and a hazard is assumed from the application setting rather than measured. These limitations do not undermine the central dissociation, which is robust across the regimes tested, but they bound the range over which the reported numbers should be read as claims about a physical system.

\subsection{Future Research}
\label{sec:disc_future}

The limitations point directly to the work that would most advance this line. The first and most promising experiment is to repeat the ablation against a realistically degraded tactile channel, incorporating dropout, deflection, and range and contact limits, and on irregular rather than rectilinear walls. The falsifiable hypothesis, supported in direction by the feeler-range result, is that the aerodynamic channel's contribution rises as the tactile channel is degraded toward physical realism, and that a regime exists in which it becomes necessary rather than redundant; resolving this requires a properly powered, multi-seed study concentrated near the degradation levels where the two channels begin to diverge. The second is to test structural uncertainty in the near-wall law by comparing alternative functional forms, which would establish whether the sensing behaviour is a property of the wall interaction broadly or of the single form assumed. The third, and the one that would most raise the standing of the work, is a physical measurement: a tethered flapping-wing mechanism beside a movable wall, with the wall-induced roll disturbance measured against real sensor noise across distance and compared with the model. This would answer the question the simulation cannot, namely whether the signal exists at a usable magnitude on hardware, and would determine whether the modelled effect rests on a real physical signal or only on an assumed one. Together these three experiments would convert the present, honestly bounded negative result into a decisive statement about when, if ever, a flapping wing's own aerodynamics are worth using to see.

\section{Conclusion}
\label{sec:conclusion}

This study investigated whether a flapping wing's near-wall aerodynamics can serve as a useful proximity sense for a confined-space micro air vehicle, using a physics-based simulation of a $77.4$~mg flapping-wing flyer controlled by a classical linear-quadratic-Gaussian regulator with a disturbance observer, and no learned policy, map, or external positioning. The wall-induced roll disturbance, recovered at no additional sensing cost from the controller's own observer, was shown to be a well-behaved proximity instrument within the adopted model, with a usable range of roughly $8$ to $40$~mm, a sub-millimetre noise-equivalent distance, a stable observer gain, and a clean two-wall response. Built on this signal together with forward-facing feelers, a purely reactive controller navigated confined corridors and detected isolated wall openings from about $12$~mm, localizing those up to $35$~mm long to within about a millimetre in a straight corridor. Across forty randomly generated multi-turn courses it reached the goal on $35$, without a wingtip strike on $30$, and detected $43$ of the $68$ openings with a mean localization error of $7.9$~mm.

The study's principal finding, however, was not the capability but the attribution of it. A controlled ablation, subjected to a pre-registered paired analysis on a corrected simulation harness, found that removing the aerodynamic channel produced no detectable change in navigation or inspection performance, while removing the tactile channel collapsed the task. The confidence interval on the central comparison, $[-5.0, +6.5]$ percentage points, does not establish formal equivalence, so a modest benefit cannot be excluded. Within that bound, the wall-induced signal is finely measurable and yet, in the conditions tested, was not shown to contribute to the task, the most likely reason being that the tactile channel already supplies what the reactive controller needs. This dissociation between a signal that can be measured and a signal that contributes was detectable only because the central comparison was made by ablation rather than by evaluating the complete system alone, and it stands as a cautionary result for sensor design at this scale.

This finding is nonetheless conditional, observed against an idealized tactile channel on rectilinear corridors, and the one regime in which an aerodynamic advantage appeared, a range-limited feeler, though not statistically established, points in the direction the physical reasoning predicts. Whether the aerodynamic sense recovers a useful role under realistic tactile degradation, whether the modelled near-wall law holds in a different functional form, and whether the signal exists at a usable magnitude on hardware are the three questions that would decide the matter, and they define the natural continuation of this work.

\section*{Data availability}

The complete simulation framework, controller, experiment scripts, and generated datasets are available at \url{https://github.com/Ashfak-Kausik/flapping-flyer}, with each reported result reproducible from a frozen repository state.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BIBLIOGRAPHY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\bibliographystyle{elsarticle-num}
\bibliography{references}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\end{document}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
