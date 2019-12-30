%!TEX root = example.tex
%*******************************************************************************
% * Copyright (c) 2006-2013 
% * Institute of Automation, Dresden University of Technology
% * 
% * All rights reserved. This program and the accompanying materials
% * are made available under the terms of the Eclipse Public License v1.0 
% * which accompanies this distribution, and is available at
% * http://www.eclipse.org/legal/epl-v10.html
% * 
% * Contributors:
% *   Institute of Automation - TU Dresden, Germany 
% *      - initial API and implementation
% ******************************************************************************/

\def\fileversion{0.1}
\def\filedate{2013/07/05}
\def\filename{ifathesis}

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{\filename}[\filedate DA/SA-Klasse des Instituts fuer Automatisierungstechnik, (c) by Stefan Hennig, Arne Sonnenburg, Matthias Freund, Christopher Martin, Lukas Baron]
\typeout{Class: '\filename' Version \fileversion, Standard DA/StA-Klasse des Instituts fuer Automatisierungstechnik.}

\newif\if@useListOfFigures\@useListOfFiguresfalse
\DeclareOption{listoffigures}{\@useListOfFigurestrue}

\newif\if@useListOfTables\@useListOfTablesfalse
\DeclareOption{listoftables}{\@useListOfTablestrue}

\newif\if@useListOfListings\@useListOfListingsfalse
\DeclareOption{listoflistings}{\@useListOfListingstrue}

\newif\if@useAbbrevations\@useAbbrevationsfalse
\DeclareOption{abbrevations}{\@useAbbrevationstrue}

\newif\if@isReadyForPrint\@isReadyForPrinttrue
\DeclareOption{print}{\@isReadyForPrinttrue}
\DeclareOption{screen}{\@isReadyForPrintfalse}

\newif\if@isIfaLogo\@isIfaLogotrue
\DeclareOption{noIfaLogo}{\@isIfaLogofalse}

\newif\if@isAFivePaper\@isAFivePaperfalse
\DeclareOption{a5paper}{\@isAFivePapertrue}

\newcommand*{\@ifaThesisLanguage}{de}
\DeclareOption{langDE}{\renewcommand*{\@ifaThesisLanguage}{de}}
\DeclareOption{langEN}{\renewcommand*{\@ifaThesisLanguage}{en}}

\newcommand*{\@ifaBibliographyStyle}{}
\DeclareOption{bibIfa}{\renewcommand*{\@ifaBibliographyStyle}{authoryear-comp}}
\DeclareOption{bibHarvard}{\renewcommand*{\@ifaBibliographyStyle}{alphabetic}}
\DeclareOption{bibNumeric}{\renewcommand*{\@ifaBibliographyStyle}{numeric-comp}}

\newcommand*{\@ifaBibliographyBackend}{bibtex8}
\DeclareOption{bibtex}{\renewcommand*{\@ifaBibliographyBackend}{bibtex}}
\DeclareOption{bibtex8}{\renewcommand*{\@ifaBibliographyBackend}{bibtex8}}
\DeclareOption{biber}{\renewcommand*{\@ifaBibliographyBackend}{biber}}

\ProcessOptions\relax

% Lade Basisklasse der Vorlage
\PassOptionsToPackage{%
  \if@isAFivePaper paper=a5\else paper=a4\fi, % Papiergröße
  headinclude=true,                           % Gehört Kopfzeile eher zur Seite oder zum Rand?
  footinclude=false,                          % Gehört Fußzeile eher zur Seite oder zum Rand?
  pagesize                                    % Schreibt die Seitengröße ins Seitenregister,
                                              % sodass DVI- und PDF-Seiten die richtige Größe haben.
}{typearea}

\PassOptionsToPackage{svgnames}{xcolor}  % Namensraum für xcolor Paket
      
\PassOptionsToPackage{fleqn}{amsmath}

% Grundschriftgröße
\newcommand{\FONT}{
  \if@isAFivePaper 9pt\else 12pt\fi
}

% Binderandgröße
\newcommand{\BCOR}{%                                        
  \if@isReadyForPrint%    
    \if@isAFivePaper 10mm\else 15mm\fi                                                                
  \else%                                                    
    0mm      
  \fi
}

% Teil der Seite, der als Seitenrand berücksichtigt wird (auch =calc mgl.):
% Z.B. DIV=10: Randbreite Oben und Mitte 1/10, Unten und Außen 2/10 der Seite.
% Regel: 60 bis 70 Zeichen pro Zeile, weniger ist besser, niemals mehr als 80!
\newcommand{\DIV}{%                                         
  \if@isReadyForPrint%                                      
    \if@isAFivePaper 13\else 10\fi                          
  \else%                                                    
    \if@isAFivePaper 11\else 9\fi         
  \fi
}

\LoadClass[
  fontsize=\FONT,                                           % Grundschriftgröße
  BCOR=\BCOR,                                               % Binderandgröße
  DIV=\DIV,                                                 % Seitenaufteilung
  captions=tableheading,                                    % Tabellenabstände für TabellenÜBERschriften optimieren
  \if@isReadyForPrint  twoside=true\else twoside=false\fi,  % Dokument zweiseitig?
  headings=small,                                           % Relative Größe der Überschriften, small ist besser lesbar und erzeugt weniger Zweizeiligs
  fleqn,                                                    % Formeln linksbündig
  bibliography=totoc,                                       % Literaturverzeichnis im Inhaltsverzeichnis referenziert
  numbers=noenddot                                         % Punkte nach Abschnittsnummern? (Weglassen = Automatik)
                                                            % Nach Duden im vorliegenden Bsp. falsch, da auch Buchstaben in Gliederung (Anhang), sieht aber besser aus.
]{scrbook}                                                  % Basisklasse der Vorlage: Koma-Skrip Klasse analog zur book-Klasse.

\addtokomafont{chapterprefix}{\mdseries}
\if@isAFivePaper
  \setkomafont{chapterprefix}{\huge}
  \setkomafont{chapter}{\Huge}
\fi

% ==============================================================
% = Definiere mögliche Parameter zur Konfiguration der Vorlage =
% ==============================================================
%%%%%
% Jetzt müssen zunächst die variablen Angaben geladen werden
\newcommand*{\@ifaThesis}{Diplomarbeit} % Type of the thesis
\newcommand*{\@ifaDissertationStage}{Gutachten} % Only used in 'Dissertation' mode to indicate the stage of the work ('Gutachten' oder 'Pflichtexemplare')
\newcommand*{\@ifaAuthor}{Max Mustermann}
\newcommand*{\@ifaAuthorBirthday}{01.01.1970}
\newcommand*{\@ifaAuthorBirthplace}{Dresden}
\newcommand*{\@ifaAuthorCourse}{Informationssystemtechnik}
\newcommand*{\@ifaAuthorYearOfMatriculation}{2000}
\newcommand*{\@ifaKeywords}{Komma-separierte, Liste, mit, Schlagwörtern, zum Thema, der Arbeit}
\newcommand*{\@ifaTitleDE}{Titel der Arbeit}
\newcommand*{\@ifaTitleEN}{Thesis' title goes here}
\newcommand*{\@ifaChair}{} % Wird nur im Modus 'Dissertation' zur Angabe des Vorsitzenden der Prüfungskommission verwendet
\newcommand*{\@ifaSupervisorA}{}
\newcommand*{\@ifaSupervisorB}{}
\newcommand*{\@ifaSupervisorC}{}
\newcommand*{\@ifaSupervisorD}{}
\newcommand*{\@ifaSupervisorE}{}
\newcommand*{\@ifaProfessor}{Titel der Arbeit}
\newcommand*{\@ifaDayOfSubmission}{\today}
\newcommand*{\@ifaDayOfDefense}{} % Wird nur im Modus 'Dissertation' zur Angabe des Tags der Verteidigung verwendet
\newcommand*{\@ifaTopicDescriptionPDF}{he0ohDie}
\newcommand*{\@ifaAppendix}{GoHaigh2}
\newcommand*{\@ifaAbstractDE}{GoHaigh2}
\newcommand*{\@ifaAbstractEN}{GoHaigh2}
\newcommand*{\@ifaReferences}{IPhei0Ox}
\newcommand*{\@ifaAbbrev}{example_files/00_abbrev}
\newcommand*{\@ifaUserListings}{}
\newcommand*{\@ifaAcknowledgements}{}
\newcommand*{\@ifaBibliographyBeforeAppendix}{true}
\newcommand*{\@ifaAdditionalContributors}{}
\newcommand*{\@ifaCV}{cv}
\newcommand*{\@ifaIncludeBeforeTitlePage}{} % Can be used for 'Dissertations' to include additional content before the title page (e.g. for a bastard title or an impressum)
\newcommand*{\@ifaIncludeAfterTitlePage}{} % Can be used for 'Dissertations' to include additional content between the title page and abstract (e.g. for an impressum)
\newcommand*{\@ifaIncludeAtEndOfDocument}{} % Can be used for 'Dissertations' to include additional content at the end of the document (e.g. to achieve an even number of pages)

\newcommand*{\@logo}{IfA_Logo_SW}
\newcommand*{\@institute}{Institut f{\"u}r Automatisierungstechnik}
\newcommand*{\@department}{Fakult{\"a}t Elektrotechnik und Informationstechnik}


\newcommand*{\ifaThesis}[1]{\renewcommand*{\@ifaThesis}{#1}}
\newcommand*{\ifaDissertationStage}[1]{\renewcommand*{\@ifaDissertationStage}{#1}}
\newcommand*{\ifaAuthor}[1]{\renewcommand*{\@ifaAuthor}{#1}}
\newcommand*{\ifaAuthorBirthday}[1]{\renewcommand*{\@ifaAuthorBirthday}{#1}}
\newcommand*{\ifaAuthorBirthplace}[1]{\renewcommand*{\@ifaAuthorBirthplace}{#1}}
\newcommand*{\ifaAuthorCourse}[1]{\renewcommand*{\@ifaAuthorCourse}{#1}}
\newcommand*{\ifaAuthorYearOfMatriculation}[1]{\renewcommand*{\@ifaAuthorYearOfMatriculation}{#1}}
\newcommand*{\ifaKeywords}[1]{\renewcommand*{\@ifaKeywords}{#1}}
\newcommand*{\ifaTitleDE}[1]{\renewcommand*{\@ifaTitleDE}{#1}}
\newcommand*{\ifaTitleEN}[1]{\renewcommand*{\@ifaTitleEN}{#1}}
\newcommand*{\ifaChair}[1]{\renewcommand*{\@ifaChair}{#1}}
\newcommand*{\ifaSupervisorA}[1]{\renewcommand*{\@ifaSupervisorA}{#1}}
\newcommand*{\ifaSupervisorB}[1]{\renewcommand*{\@ifaSupervisorB}{#1}}
\newcommand*{\ifaSupervisorC}[1]{\renewcommand*{\@ifaSupervisorC}{#1}}
\newcommand*{\ifaSupervisorD}[1]{\renewcommand*{\@ifaSupervisorD}{#1}}
\newcommand*{\ifaSupervisorE}[1]{\renewcommand*{\@ifaSupervisorE}{#1}}
\newcommand*{\ifaProfessor}[1]{\renewcommand*{\@ifaProfessor}{#1}}
\newcommand*{\ifaDayOfSubmission}[1]{\renewcommand*{\@ifaDayOfSubmission}{#1}}
\newcommand*{\ifaDayOfDefense}[1]{\renewcommand*{\@ifaDayOfDefense}{#1}}
\newcommand*{\ifaTopicDescriptionPDF}[1]{\renewcommand*{\@ifaTopicDescriptionPDF}{#1}}
\newcommand*{\ifaAppendix}[1]{\renewcommand*{\@ifaAppendix}{#1}}
\newcommand*{\ifaAbstractDE}[1]{\renewcommand*{\@ifaAbstractDE}{#1}}
\newcommand*{\ifaAbstractEN}[1]{\renewcommand*{\@ifaAbstractEN}{#1}}
\newcommand*{\ifaReferences}[1]{\renewcommand*{\@ifaReferences}{#1}}
\newcommand*{\ifaAcknowledgements}[1]{\renewcommand*{\@ifaAcknowledgements}{#1}}
\newcommand*{\ifaAbbrev}[1]{\renewcommand*{\@ifaAbbrev}{#1}}
\newcommand*{\ifaUserListings}[1]{\renewcommand*{\@ifaUserListings}{#1}}
\newcommand*{\ifaBibliographyBeforeAppendix}[1]{\renewcommand*{\@ifaBibliographyBeforeAppendix}{#1}}
\newcommand*{\ifaAdditionalContributors}[1]{\renewcommand*{\@ifaAdditionalContributors}{#1}}
\newcommand*{\ifaCV}[1]{\renewcommand*{\@ifaCV}{#1}}
\newcommand*{\ifaIncludeBeforeTitlePage}[1]{\renewcommand*{\@ifaIncludeBeforeTitlePage}{#1}}
\newcommand*{\ifaIncludeAfterTitlePage}[1]{\renewcommand*{\@ifaIncludeAfterTitlePage}{#1}}
\newcommand*{\ifaIncludeAtEndOfDocument}[1]{\renewcommand*{\@ifaIncludeAtEndOfDocument}{#1}}
\newcommand*{\otherInstitute}[1]{\renewcommand*{\@institute}{#1}}
\newcommand*{\otherDepartment}[1]{\renewcommand*{\@department}{#1}}
\newcommand*{\otherLogo}[1]{\renewcommand*{\@logo}{#1}}



% Schriftspezifische Definitionen (wird lediglich für die Kopfzeile verwendet)
\RequirePackage[scale=.95]{opensans} %scales the letter spacing, default refers to the printing style for numbers
\DeclareFixedFont{\openSansLightIX}{T1}{fos}{l}{n}{\if@isAFivePaper 7\else 9\fi}
\DeclareFixedFont{\openSansBoldIX}{T1}{fos}{b}{n}{\if@isAFivePaper 7\else 9\fi}


% Standardschrift (muss manuell zurückgesetzt werden, nachdem oben das 'OpenSans' package geladen wurde)
\renewcommand*\familydefault{cmr} % computer modern roman

%-------------------------------------------------------------------------
\RequirePackage{ifthen}

% Paket für einfache Realisation von String-Vergleichen, die hier in der Vorlage verwendet werden
\RequirePackage{xstring}

% Absatznummerierung fuer 4 Ebenen (z.B. 2.3.2.1) gemaess IfA Richtlinie einschalten.
\setcounter{secnumdepth}{3}
\setcounter{tocdepth}{3} 

\RequirePackage{scrhack} %Zur Herstellung der Kompabilitaet mit Paketen, die nicht kompatibel zum KOMA-Script sind.

% Muss 'atEndOfClass' geladen werden, da es sonst zu Problemen kommt (bibtex/biber meldet, es könnten Verweise nicht aufgelöst werden, obwohl dies nicht der Fall ist)
\AtEndOfClass{
  \RequirePackage[           % Verwendung von biblatex für das Literaturverzeichnis
    style=\@ifaBibliographyStyle, 
    backend=\@ifaBibliographyBackend, 
    maxbibnames=99,
    backref=true,
    natbib=true
  ]{biblatex}

  % adds the additional driver for @standard bibliography entries
  \DeclareBibliographyDriver{standard}{%
    \usebibmacro{bibindex}%
    \usebibmacro{begentry}%
    \usebibmacro{author}%
    \setunit{\labelnamepunct}\newblock
    \usebibmacro{title}%
    \newunit\newblock
    \printfield{number}%
    \setunit{\addspace}\newblock
    \printfield[parens]{type}%
    \newunit\newblock
    \usebibmacro{location+date}%
    \newunit\newblock
    \iftoggle{bbx:url}
      {\usebibmacro{url+urldate}}
      {}%
    \newunit\newblock
    \usebibmacro{addendum+pubstate}%
    \setunit{\bibpagerefpunct}\newblock
    \usebibmacro{pageref}%
    \newunit\newblock
    \usebibmacro{related}%
    \usebibmacro{finentry}%
  }  

  % Erlaube URL-Linebreaks
  \setcounter{biburlnumpenalty}{6000}% Penalty für Zahlenumbruch
  \setcounter{biburlucpenalty}{3000} % Penalty für Umbruch an Großbuchstaben
  \setcounter{biburllcpenalty}{9000} % Penalty für Umbruch an Kleinbuchstaben

  \interfootnotelinepenalty=9999 % Verhindert, dass Fußnoten auf zwei Seiten verteilt werden
}


% Lade weitere benötigte Pakete
\include{packages}

% definiere Farben
\definecolor{TUDBLAU}{rgb}{0.0,0.348,0.637}
\definecolor{OSGREEN}{rgb}{0,0.75,0.25} 

% Kläre zunächst einige Dinge, die von der Sprache abhängen
\ifthenelse{\equal{\@ifaThesisLanguage}{de}}{ 
  % Deutsche Pakete
  \RequirePackage[ngerman]{babel}    % Silbentrennung nach neuer deutscher Rechtschreibung.
  \RequirePackage[german]{nomencl}
  % Deutsche Begrifflichkeiten
  \newcommand{\@conTitle}{Inhaltsverzeichnis}
  \newcommand{\@bibTitle}{Literaturverzeichnis}
  \newcommand{\@appTitle}{Anhang}
  \renewcommand{\@pdftitle}{\@ifaTitleDE}
  \renewcommand{\nomname}{Abkürzungs- und Symbolverzeichnis}
  \renewcommand{\lstlistlistingname}{Quelltextverzeichnis}            
}{
  % Englische Pakete
  \RequirePackage[english]{babel}    % Silbentrennung nach englischer Rechtschreibung.
  \RequirePackage[english]{nomencl}
  % Englische Begrifflichkeiten
  \newcommand{\@conTitle}{Contents}
  \newcommand{\@bibTitle}{References}
  \newcommand{\@appTitle}{Appendix}
  \renewcommand{\@pdftitle}{\@ifaTitleEN}
  \renewcommand{\nomname}{Nomenclature}
  \renewcommand{\lstlistlistingname}{List of Listings}          
}

%-------------------------------------------------------------------------
% Inserts the typical IfA header including the TU and IfA logo as well as
% the faculty and institute name
%-------------------------------------------------------------------------
\newcommand{\@ifaPrintHeader}{%
  \begin{center}
    \vspace*{\if@isAFivePaper -18.83mm\else -26.63mm\fi} % height of the logo (16.73mm) + skip below the logo (3.75mm) + one line (approx. 6.15mm)
    \begin{tabular*}{\textwidth}[b]{@{}p{\textwidth}@{}}%@{\extracolsep\fill}
      % TUD-Logo
        \hspace*{\if@isAFivePaper -13.44mm\else -19mm\fi}\includegraphics[height=\if@isAFivePaper 11.83mm\else 16.73mm\fi]{TU_Logo_SW}%  width=57mm
        \hfill
      % IfA-Logo
        \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{ % According to Frau Wermuth, there must only be the TU logo on the title page of a dissertation
          \if@isIfaLogo   \includegraphics[height=\if@isAFivePaper 11.83mm\else 16.73mm\fi]{\@logo}\fi
        }{}
        \\[\if@isAFivePaper 2.65mm\else 3.75mm\fi]
        \hline%
      % Einrichtungsname
        \rule[-1.2mm]{0mm}{\if@isAFivePaper 11pt\else 13pt\fi}%\openSansBoldIX 
        \fosfamily\textbf {\fontsize{\if@isAFivePaper 7\else 9\fi}{\if@isAFivePaper 9\else 12\fi}\selectfont \@department}%
        % \usefont{OT1}{fos}{b}{n}\fontsize{\if@isAFivePaper 7\else 9\fi}{\if@isAFivePaper 9\else 12\fi}\selectfont Fakult{\"a}t Elektrotechnik und Informationstechnik%
        % \openSansBoldIX Fakult{\"a}t Elektrotechnik und Informationstechnik%
      % Institut
        \ 
        % \openSansLightIX Institut f{\"u}r Automatisierungstechnik%
        \fosfamily {\fontseries{l}\fontsize{\if@isAFivePaper 7\else 9\fi}{\if@isAFivePaper 9\else 12\fi}\selectfont \@institute}%
        \\
        \hline
    \end{tabular*}%
  \end{center}
}

%-------------------------------------------------------------------------
% Macht im Anhang aus "A Quellcode" -> "Anhang A Quellcode"
%-------------------------------------------------------------------------
\newcommand*{\appendixmore}{%
  \renewcommand*{\chapterformat}{%
    \appendixname~\thechapter\autodot:\enskip}
  \renewcommand*{\chaptermarkformat}{%
    \appendixname~\thechapter\autodot:\enskip}
}
%-------------------------------------------------------------------------
                                              
% Veränderung der Bildunterschriften und Tabellenüberschriften:
\addtokomafont{caption}{\small}                 % Kleinere Schriftgröße für Captions
\setkomafont{captionlabel}{\sffamily\bfseries}  % Caption-Label fett und serifenlos
\setcapindent{1em}                              % 2. Zeile der Caption mit hängendem Einzug

%-------------------------------------------------------------------------
% Anpassung des Literaturverzeichnisses
%-------------------------------------------------------------------------
\AtEndOfClass{ % muss 'atEndOfClass' ausgeführt werden, da das 'biblatex'-Paket erst dann geladen wird (s.o.)
  \setlength{\bibitemsep}{10pt}
  \defbibheading{bibliography}{
        \chapter*{\@bibTitle}
        \addcontentsline{toc}{chapter}{\@bibTitle}
  }
  % Notwendig für Autovervollständigung der Zitate im Texnic Center
  \renewcommand*{\bibliography}[1]{\addbibresource{#1.bib}}
  % Anpassung für eckige Klammern bei 'authoryear-comp'
  \ifthenelse{\equal{\@ifaBibliographyStyle}{authoryear-comp}}{\renewcommand*{\citep}[2][]{\mkbibbrackets{\cite[#1]{#2}}}}{}
}

%-------------------------------------------------------------------------
% Löst Probleme mit deutschen Umlauten in Listings
%-------------------------------------------------------------------------
\lstset{literate=%
  {Ö}{{\"O}}1
  {Ä}{{\"A}}1
  {Ü}{{\"U}}1
  {ß}{{\ss}}2
  {ü}{{\"u}}1
  {ä}{{\"a}}1
  {ö}{{\"o}}1
}

%-------------------------------------------------------------------------
% Integriert Listings und baut diese als Gleitobjekt aus, wenn gewünscht
%-------------------------------------------------------------------------
% Usage: \listing{<Caption>}{<Label>}{<[Dialect]Language>}{<Pfad>}{<float?>} 
\captionsetup[lstlisting]{margin={0pt,-.3cm},singlelinecheck=true,labelsep=space}%Fixing the listing caption formatting that originally is centered to the top-line of the listing
\newcommand*{\ifalisting}[6]{
  \renewcommand{\lstlistingname}{Listing}
  \ifthenelse{\equal{#6}{true}}{
    \ifalistingFloat{#1}{#2}{#3}{#4}{#5}
  }{
    \ifalistingNoFloat{#1}{#2}{#3}{#4}{#5}
  }  
  \vspace{4.5mm}
}

\newcommand*{\ifalistingFloat}[5]{% 
    \begin{minipage}{0.96\textwidth}
    \lstset{belowcaptionskip=2mm}                   % Abstand unterhalb der Beschriftung
    \ifalistingRaw{#1}{#2}{#3}{#4}{#5}
    \end{minipage}
}
  
% Thanks to Matthias Freund who provided the code to enables listings which are longer than one page.
%                                  
\newcommand*{\ifalistingNoFloat}[5]{%  
  \begin{addmargin}[4mm]{1.5mm}       % verändert den Abstand ohne Verwendung einer Minipage
    \lstset{belowcaptionskip=2mm}                  % Abstand unterhalb der Beschriftung
    \ifalistingRaw{#1}{#2}{#3}{#4}{#5}
    \vspace{-1mm}
    \begin{flushright}
      \scriptsize
      Listing \ref{#2}
    \end{flushright}
  \end{addmargin}
}                                      

\newcommand*{\ifalistingRaw}[5]{
  \lstinputlisting[
    caption={#1},
    label={#2}, 
    language={#3},
    keywordstyle=\color{blue},
    identifierstyle=\color{black},
    stringstyle=\itshape\color{red},
    basicstyle=\ttfamily\color{darkgray}\footnotesize,% Schriftschnitt des Codes
    showstringspaces=false,                           % Leerzeichen nicht markieren
    commentstyle=\ttfamily\color{teal},               % Schriftschnitt der Kommentare
    tabsize=2,                                        % Tabulatorengröße in Leerzeichen
    breaklines=true,                                  % zu lange Zeilen umbrechen
    frame=tb,                                         % Rahmen um das Listing
    abovecaptionskip=3mm,                             % Abstand oberhalb der Beschriftung
    captionpos=t,                                     % Position der Beschriftung (b=bottom, t=top)
    numbers=#4,
    numbersep=5pt
  ]{#5}
}

% Erlaubt das Einfügen eines automatischen Symbolverzeichnisses
% Definiere die richtige Sprache (bspw. für Silbentrennung, ...)
% [Die entsprechenden Pakete wurden bereits eingefügt]
\makenomenclature    

% Jetzt geht es los...
\AtBeginDocument{%
  % colorlinks: Farbige Verweise statt Umrahmung 
  % Verhalten von hyperef Ändern: Farbe für Bildschirm, SW für Druck
  % ...color: Farbe verschiedener Verweise nach Farbpalette svgnames
  % pdf...: Setzten von Variablen die in den Eigenschaften der PDF-Datei angzeigt werden
  % Weitere Optionen möglich - siehe Dokumentation des Paktes hyperref  
  \hypersetup{
    linkcolor=\if@isReadyForPrint black\else TUDBLAU\fi,%
    urlcolor=\if@isReadyForPrint black\else TUDBLAU\fi,%
    citecolor=\if@isReadyForPrint black\else OSGREEN\fi,%
    pdftitle=\@pdftitle,%
    pdfauthor=\@ifaAuthor,%
    pdfsubject=\@pdftitle,%
    pdfkeywords={\@ifaKeywords}%,%
  }
  
  % Check paper size
  \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{%
    \if@isAFivePaper %
      \ClassWarning{ifathesis}{%
        Klassenoption 'a5paper' sollte nur bei Dissertationen gewählt werden. Studienarbeiten, Diplomarbeiten, etc. sind grundsätzliche auf A4 zu drucken!
      }%
    \fi%
  }{
    % Show chapter prefixes if in 'Pflichtexemplare' mode:
    % % 'Kapitel 1\nKapitelname' instead of '1. Kapitelname'?
    \ifthenelse{\equal{\@ifaDissertationStage}{Pflichtexemplare}}{\KOMAoption{chapterprefix}{true}}{}
  }

  \frontmatter

% ================================================
% = Bastard title, etc. (only for dissertations) =
% ================= ==============================

  \ifthenelse{\equal{\@ifaThesis}{Dissertation}}{
    \ifthenelse{\equal{\@ifaDissertationStage}{Pflichtexemplare}}{
      \IfFileExists{\@ifaIncludeBeforeTitlePage}{
        \pagestyle{empty}
        \input{\@ifaIncludeBeforeTitlePage}
        \cleardoublepage
      }{}
    }{}
  }{}

% =================
% = Titelseite(n) =
% =================  
  \begin{titlepage}

    \@ifaPrintHeader
    
    % A dissertation will get a different titlepage
    \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{
      % ===============================
      % = Diplom-, Studienarbeit etc. =
      % ===============================
      \begin{center}
        \vfill
        \vfill
        {\huge\textsc{\@ifaThesis}}\\[0.5cm]
          zum Thema \\[0.5cm]
        {\large \IfStrEq{\@ifaThesisLanguage}{de}{\@ifaTitleDE}{\@ifaTitleEN}}\\
        \vfill
        \begin{tabular}{rl}
          vorgelegt von & \@ifaAuthor \\
          im Studiengang & \@ifaAuthorCourse,\:Jg.\:\@ifaAuthorYearOfMatriculation \\
          geboren am & \@ifaAuthorBirthday\:in\:\@ifaAuthorBirthplace\\ 
        \end{tabular}
        \vfill
        \ifthenelse{\equal{\@ifaThesis}{Diplomarbeit}}{
          {\large zur Erlangung des akademischen Grades eines} \\[0.2cm]
          {\Large Diplomingenieurs\\[0.2cm](Dipl.-Ing.)}
          \vfill
        }{
          \vfill
          \vfill
        }
        \begin{tabular}{rl}
          Betreuer: \ifthenelse{\equal{\@ifaSupervisorA}{}}{}{&\@ifaSupervisorA\\ \ifthenelse{\equal{\@ifaSupervisorB}{}}{}{&\@ifaSupervisorB\\ \ifthenelse{\equal{\@ifaSupervisorC}{}}{}{&\@ifaSupervisorC\\\ifthenelse{\equal{\@ifaSupervisorD}{}}{}{&\@ifaSupervisorD\\ \ifthenelse{\equal{\@ifaSupervisorE}{}}{}{&\@ifaSupervisorE\\}}}}}
          Verantwortlicher Hochschullehrer:   & \@ifaProfessor \\
          Tag der Einreichung:  & \@ifaDayOfSubmission \\
        \end{tabular}
      \end{center}
    }{
      % ================
      % = Dissertation =
      % ================
      \begin{center}
        \vfill
        \vfill
        \setstretch{1.7}
        % The german title always needs to be the 'main' title (disregarding the actual language of the dissertation). 
        {\if@isAFivePaper \LARGE\else \Large\fi {\bfseries\@ifaTitleDE}}
        % However, if the language of the dissertation is english, we also add the english title as 'secondary' title.
         \IfStrEq{\@ifaThesisLanguage}{de}{}{%
          \\[5mm]
          {\if@isAFivePaper \LARGE\else \Large\fi \@ifaTitleEN}
        }
        \\[5mm]
        \setstretch{1}
        \vfill
        {\if@isAFivePaper \Large\else \large\fi\bfseries\@ifaAuthor}
        \vfill
        \vfill
        \ifthenelse{\equal{\@ifaDissertationStage}{Pflichtexemplare}}{von }{}der Fakultät Elektrotechnik und Informationstechnik\\
        der Technischen Universität Dresden\\[5mm]
        zur Erlangung des akademischen Grades eines\\[2mm]
        {\if@isAFivePaper \LARGE\else \Large\fi{\bfseries Doktoringenieurs}\\[0.2cm](Dr.-Ing.)}\\[5mm]
        \ifthenelse{\equal{\@ifaDissertationStage}{Pflichtexemplare}}{genehmigte }{vorgelegte }Dissertation\\[5mm]
        ------ \ding{91} ------
        \vfill
        \begin{tabular}{rl}
          % Only print the 'Vorsitzender', 'Gutachter', and 'Tag der Verteidigung' during the 'Pflichtexemplare' stage
          \ifthenelse{\equal{\@ifaDissertationStage}{Gutachten}}{}{%
            \ifthenelse{\equal{\@ifaChair}{}}{%
              \ClassWarning{ifathesis}{%
                Kein(e) Vorsitzende(r) der Promotionskommission angegeben. Dies ist fuer die Pflichtexemplare der Dissertation unbedingt erforderlich.
              }% 
            }{%
              Vorsitzender: & \@ifaChair \vspace*{.7em}\\%
            }%
          }%
          \ifthenelse{\equal{\@ifaDissertationStage}{Gutachten}}{}{%
            \ifthenelse{\equal{\@ifaSupervisorA}{}}{\ClassWarning{ifathesis}{Kein(e) erste(r) Gutachter(in) angegeben. Dies ist fuer die Pflichtexemplare der Dissertation unbedingt erforderlich.}}{}%
            \ifthenelse{\equal{\@ifaSupervisorB}{}}{\ClassWarning{ifathesis}{Kein(e) zweite(r) Gutachter(in) angegeben. Dies ist fuer die Pflichtexemplare der Dissertation unbedingt erforderlich.}}{}%
            Gutachter: \ifthenelse{\equal{\@ifaSupervisorA}{}}{}{&\@ifaSupervisorA\vspace*{.1em}\ifthenelse{\equal{\@ifaSupervisorB}{}}{}{\\&\@ifaSupervisorB\vspace*{.1em}\ifthenelse{\equal{\@ifaSupervisorC}{}}{}{\\&\@ifaSupervisorC\vspace*{.1em}\ifthenelse{\equal{\@ifaSupervisorD}{}}{}{\\&\@ifaSupervisorD\vspace*{.1em}\ifthenelse{\equal{\@ifaSupervisorE}{}}{}{\\&\@ifaSupervisorE\vspace*{.1em}}}}}}\vspace*{.6em}\\%
          }
          Tag der Einreichung: & \@ifaDayOfSubmission\vspace*{.7em}\\
          \ifthenelse{\equal{\@ifaDissertationStage}{Gutachten}}{}{%
            \ifthenelse{\equal{\@ifaDayOfDefense}{}}{%
              \ClassWarning{ifathesis}{%
                Kein Tag der Verteidigung angegeben. Dies ist fuer die Pflichtexemplare der Dissertation unbedingt erforderlich.
              }% 
            }{%
             Tag der Verteidigung: & \@ifaDayOfDefense \\%
            }%
          }%
        \end{tabular}
      \end{center}
    }
  \end{titlepage}

% ================================================
% = Impressum, etc. (only for dissertations) =
% ================= ==============================

  \ifthenelse{\equal{\@ifaThesis}{Dissertation}}{
    \ifthenelse{\equal{\@ifaDissertationStage}{Pflichtexemplare}}{
      \clearpage
      \IfFileExists{\@ifaIncludeAfterTitlePage}{
        \pagestyle{empty}
        \input{\@ifaIncludeAfterTitlePage}
      }{}
    }{}
  }{}

  \cleardoublepage

  \begingroup
    \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{
      % Im Vorspann nur Seiten ohne Kopf- und Fußzeile (außer Abschnittsbeginn)
      \pagestyle{empty}
      % Auch Abschnittsbeginn ohne Kopf- und Fußzeile
      \renewcommand*{\chapterpagestyle}{empty}
    }{
      % Im Vorspann werden kleine römische Seitenzahlen genutzt
      \pagestyle{scrheadings}
    }

    % Aufgabenstellung
    \IfFileExists{\@ifaTopicDescriptionPDF} {
      \includepdf{\@ifaTopicDescriptionPDF}
      \cleardoublepage 
    }{
      \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{
        \ClassWarning{ifathesis}{Es wurde keine Aufgabenstellung angegeben.}
      }{}
    }

    % Danksagung
    \IfFileExists{\@ifaAcknowledgements}{
      \chapter*{\IfStrEq{\@ifaThesisLanguage}{de}{Danksagung}{Acknowledgements}}
      \input{\@ifaAcknowledgements}
      \cleardoublepage
    }{}
      
    % Kurzfassungen
    \IfFileExists{\@ifaAbstractDE}{
      \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{
        \@ifaPrintHeader
        \vfill
        \section*{\@ifaTitleDE}
        \input{\@ifaAbstractDE}
        \vfill
        \begin{flushleft}
          \scriptsize
          \begin{tabular}{ll}
            Betreuer: \ifthenelse{\equal{\@ifaSupervisorA}{}}{}{&\@ifaSupervisorA\\ \ifthenelse{\equal{\@ifaSupervisorB}{}}{}{&\@ifaSupervisorB\\ \ifthenelse{\equal{\@ifaSupervisorC}{}}{}{&\@ifaSupervisorC\\ \ifthenelse{\equal{\@ifaSupervisorD}{}}{}{&\@ifaSupervisorD\\ \ifthenelse{\equal{\@ifaSupervisorE}{}}{}{&\@ifaSupervisorE\\}}}}}Hochschullehrer: &\@ifaProfessor\\Tag der Einreichung: &\@ifaDayOfSubmission \\
          \end{tabular}
          \hfill
          \hrule
          \vspace{.2cm}\normalsize

          \ifthenelse{\equal{\@ifaThesis}{Diplomarbeit}}{DIPLOMARBEIT}{}
          \ifthenelse{\equal{\@ifaThesis}{Masterarbeit}}{MASTERARBEIT}{}
          \ifthenelse{\equal{\@ifaThesis}{Bachelorarbeit}}{BACHELORARBEIT}{}
          \ifthenelse{\equal{\@ifaThesis}{Studienarbeit}}{STUDIENARBEIT}{}
          \ifthenelse{\equal{\@ifaThesis}{Final Project}}{FINAL PROJECT}{}
          \ifthenelse{\equal{\@ifaThesis}{Forschungspraktikum}}{FORSCHUNGSPRAKTIKUM}{}
          \hfill Bearbeiter: \@ifaAuthor
        \end{flushleft}
      }{
        \thispagestyle{\chapterpagestyle}
        \chapter*{Kurzfassung}
        \addcontentsline{toc}{chapter}{Kurzfassung}%
        \input{\@ifaAbstractDE}
      }
      \cleardoublepage
    }{
      % no german abstract found
      \ifthenelse{\equal{\@ifaThesis}{Dissertation}}{
        \ifthenelse{\equal{\@ifaDissertationStage}{Gutachten}}{}{%
          \ClassWarning{ifathesis}{%
            Konnte die deutsche Kurzfassung nicht finden. Diese ist fuer die Pflichtexemplare der Dissertation unbedingt
            erforderlich.
          } 
        }
      }{
        \ClassWarning{ifathesis}{
          Konnte die deutsche Kurzfassung nicht finden. Diese ist fuer Arbeiten am Institut fuer 
          Automatisierungstechnik unbedingt erforderlich. Sollte die Vorlage anderswo verwendet, 
          dann kann diese Warnung ignoriert werden.
        }
      }
    }

    % Englische Kurzfassung
    \IfFileExists{\@ifaAbstractEN}{
      \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{
        \@ifaPrintHeader
        \vfill
        \section*{\@ifaTitleEN}
        \input{\@ifaAbstractEN}
        \vfill
          \begin{flushleft}
            \scriptsize
            \begin{tabular}{ll}
              Tutor: \ifthenelse{\equal{\@ifaSupervisorA}{}}{}{&\@ifaSupervisorA\\ \ifthenelse{\equal{\@ifaSupervisorB}{}}{}{&\@ifaSupervisorB\\ \ifthenelse{\equal{\@ifaSupervisorC}{}}{}{&\@ifaSupervisorC\\ \ifthenelse{\equal{\@ifaSupervisorD}{}}{}{&\@ifaSupervisorD\\ \ifthenelse{\equal{\@ifaSupervisorE}{}}{}{&\@ifaSupervisorE\\}}}}}Supervisor: &\@ifaProfessor\\Day of Submission: &\@ifaDayOfSubmission \\
            \end{tabular}
            \hfill
            \hrule
            \vspace{.2cm}\normalsize
            \ifthenelse{\equal{\@ifaThesis}{Diplomarbeit}}{DIPLOMA THESIS}{STUDENT RESEARCH THESIS}
            \hfill Author: \@ifaAuthor
          \end{flushleft}
      }{
        {
          \thispagestyle{\chapterpagestyle}
          \chapter*{Abstract}
          \addcontentsline{toc}{chapter}{Abstract}%
          \input{\@ifaAbstractEN}
        }
      }
      \cleardoublepage
    }{
      % no english abstract found
      \ifthenelse{\equal{\@ifaThesis}{Dissertation}}{
        \ifthenelse{\equal{\@ifaDissertationStage}{Gutachten}}{}{%
          \ClassWarning{ifathesis}{%
            Konnte die englische Kurzfassung nicht finden. Diese ist fuer die Pflichtexemplare der Dissertation unbedingt
            erforderlich. 
          }
        }
      }{
        \ClassWarning{ifathesis}{
          Konnte die englische Kurzfassung nicht finden. Diese ist fuer Arbeiten am Institut fuer 
          Automatisierungstechnik unbedingt erforderlich. Sollte die Vorlage anderswo verwendet, 
          dann kann diese Warnung ignoriert werden.
        }
      }
    }
    
    % Verzeichnisse
    \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{
      \pdfbookmark[0]{\@conTitle}{toc}
    }{
      % Da für Dissertationen Kurzfassung, Abstract, LOF, etc. ins Inhaltsverzeichnis aufgenommen werden, sollte der Vollständigkeit halber auch das Inhaltsverzeichnis selbst aufgenommen werden (ansonsten entsteht eine 'Lücke')
      \setuptoc{toc}{totoc} 
    }
    \tableofcontents
    \clearpage
    \if@useListOfFigures
      \listoffigures
      \cleardoublepage
    \fi
    \if@useListOfTables
      \listoftables
      \cleardoublepage
    \fi
    \if@useListOfListings
      \lstlistoflistings
      \cleardoublepage
    \fi
    \IfFileExists{\@ifaUserListings}{
      \input{\@ifaUserListings}
      \cleardoublepage
    }{}
    \if@useAbbrevations
      \IfFileExists{\@ifaAbbrev}{
        \input{\@ifaAbbrev}
        \cleardoublepage
      }{}
    \fi
  \endgroup
  
  \mainmatter

  \pagestyle{scrheadings}
}

% ...und gleich ist Schluss!
\AtEndDocument{
  \cleardoublepage
  % Auch Abschnittsbeginn ohne Kopf- und Fußzeile
  %\renewcommand*{\chapterpagestyle}{empty} 
  % Literaturverzeichnis vor dem Anhang
  \ifthenelse{\equal{\@ifaBibliographyBeforeAppendix}{true}}{        
    \if@isReadyForPrint  
      \markboth{\@bibTitle}{}
    \else 
      \markright{\@bibTitle}
    \fi
    \printbibliography 
    \cleardoublepage
    \automark[section]{chapter}
  }{}

  % Anhänge
  \IfFileExists{\@ifaAppendix}{
    \renewcommand{\appendixpagename}{\@appTitle}
    % Zuerst die Titelseite                          
    \begin{center}                          
      \thispagestyle{empty}
      \vspace*{\fill}
    {\sffamily\bfseries\if@isAFivePaper\Huge\else\huge\fi\appendixpagename}
    \vspace{\fill}
      \cleardoublepage
    \end{center}
    % Nun die Anhänge, ohne Titelseite (siehe \RequirePackage --> ohne page-Option)
    \begin{appendices}
      \ifthenelse{\equal{\@ifaBibliographyBeforeAppendix}{true}}{        
        \pagenumbering{Roman}             % römische Nummerierung im Anhang
      \setcounter{page}{1}              % beginne wieder mit Seitenzahl 1
      }{}
      \input{\@ifaAppendix}
      \cleardoublepage
    \end{appendices}
  }{}
           
  % Literaturverzeichnis nach dem Anhang
  \ifthenelse{\equal{\@ifaBibliographyBeforeAppendix}{false}}{
    \if@isReadyForPrint  
      \markboth{\@bibTitle}{}
    \else 
      \markright{\@bibTitle}
    \fi
    \printbibliography 
    \cleardoublepage
    \automark[section]{chapter}
  }{}

  \backmatter

  % Im Abspann nur Seiten ohne Kopf- und Fußzeile (außer Abschnittsbeginn)
  \pagestyle{empty}  
  % Auch Abschnittsbeginn ohne Kopf- und Fußzeile
  \renewcommand*{\chapterpagestyle}{empty}

  \ifthenelse{\not\equal{\@ifaThesis}{Dissertation}}{
    % Selbständigkeitserklärung (nicht bei Dissertationen)
    
    \vspace*{\fill} %4cm} 
    \section*{Selbstständigkeitserklärung}

    Hiermit versichere ich, \@ifaAuthor, geboren am \@ifaAuthorBirthday~in
    \@ifaAuthorBirthplace, dass ich \ifthenelse{\not\equal{\@ifaThesis}{Forschungspraktikum}}{die vorliegende \@ifaThesis}{das vorliegende \@ifaThesis}~zum Thema
      \begin{quote}
         \begin{center}
            \emph{\@ifaTitleDE}
         \end{center}
      \end{quote}
    ohne unzulässige Hilfe Dritter und ohne Benutzung anderer als der angegebenen Hilfsmittel angefertigt habe; 
    die aus fremden Quellen direkt oder indirekt übernommenen Gedanken sind als
    solche kenntlich gemacht. Bei der Auswahl und Auswertung des Materials sowie
    bei der Herstellung des Manuskripts habe ich Unterstützungsleistungen von folgenden Personen erhalten:
          \begin{center}
          \emph{\@ifaSupervisorA\IfStrEq{\@ifaSupervisorB}{}{}{, \@ifaSupervisorB}\IfStrEq{\@ifaSupervisorC}{}{}{, \@ifaSupervisorC}\IfStrEq{\@ifaSupervisorD}{}{}{, \@ifaSupervisorD}\IfStrEq{\@ifaSupervisorE}{}{}{, \@ifaSupervisorE}\IfStrEq{\@ifaAdditionalContributors}{}{}{, \@ifaAdditionalContributors}}
        \end{center}
    Weitere Personen waren an der geistigen Herstellung \ifthenelse{\not\equal{\@ifaThesis}{Forschungspraktikum}}{der vorliegenden \@ifaThesis}{des vorliegenden \@ifaThesis s}~nicht beteiligt. Mir ist bekannt, dass die Nichteinhaltung dieser Erklärung zum nachträglichen Entzug des Diplomabschlusses (Masterabschlusses) führen kann.
    \vspace{2cm}
    \begin{flushright}
      Dresden, den \@ifaDayOfSubmission \hfill \dotfill\\
      Unterschrift
    \end{flushright}
    \vfill
  }{
    % CV (falls gewünscht, nur bei Dissertationen)
    \IfFileExists{\@ifaCV}{
      \includepdf{\@ifaCV}
    }{}
  }

% ================================================
% = Final content at the end of the file         =
% ================= ==============================

  \ifthenelse{\equal{\@ifaThesis}{Dissertation}}{
    \ifthenelse{\equal{\@ifaDissertationStage}{Pflichtexemplare}}{
      \clearpage
      \IfFileExists{\@ifaIncludeAtEndOfDocument}{
        \pagestyle{empty}
        \input{\@ifaIncludeAtEndOfDocument}
      }{}
    }{}
  }{}
  
}%EOF