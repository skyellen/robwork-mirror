;======================================================
; Include
 
  !include "MUI.nsh"
 
; The name of the installer

Name "RobWork VS2005 Installer v0.1.0"
OutFile "RobWork-VS2005-Installer_0.1.0.exe"
InstallDir c:\local\

; Registry key to check for directory (so if you install again, it will 
; overwrite the old one automatically)
InstallDirRegKey HKLM "Software\RobWorkVS2005Package" "Install_Dir"

; Request application privileges for Windows Vista
RequestExecutionLevel admin



;======================================================
; Modern Interface Configuration
 
  

  !define MUI_WELCOMEPAGE_TITLE "RobWork VS2005 package installer"

  !define MUI_HEADERIMAGE
  !define MUI_ABORTWARNING
  !define MUI_COMPONENTSPAGE_SMALLDESC
  !define MUI_HEADERIMAGE_BITMAP_NOSTRETCH
  !define MUI_FINISHPAGE
  !define MUI_FINISHPAGE_TEXT "Thank you for installing the RobWork package."
  !define MUI_WELCOMEFINISHPAGE_BITMAP "splash.jpg"
  !define MUI_ICON "RobWork.ico"
 
 
 !macro ReplaceInFile SOURCE_FILE SEARCH_TEXT REPLACEMENT
  Push "${SOURCE_FILE}"
  Push "${SEARCH_TEXT}"
  Push "${REPLACEMENT}"
  Call RIF
!macroend
 
 !macro _strReplaceConstructor OUT NEEDLE NEEDLE2 HAYSTACK
  Push "${HAYSTACK}"
  Push "${NEEDLE}"
  Push "${NEEDLE2}"
  Call StrReplace
  Pop "${OUT}"
!macroend
 
!define StrReplace '!insertmacro "_strReplaceConstructor"'
 
;======================================================
; Pages
 
  !insertmacro MUI_PAGE_WELCOME
  !insertmacro MUI_PAGE_LICENSE "workspace\RobWork\license.txt"
  !insertmacro MUI_PAGE_LICENSE "workspace\RobWorkStudio\license.txt"
  !insertmacro MUI_PAGE_COMPONENTS
  ;Page components
  ;Page instfiles
  ;Page directory
  !insertmacro MUI_PAGE_DIRECTORY
  !insertmacro MUI_PAGE_INSTFILES
  !insertmacro MUI_PAGE_FINISH

  !insertmacro MUI_UNPAGE_CONFIRM
  !insertmacro MUI_UNPAGE_INSTFILES
  
;======================================================
; Languages
 
  !insertmacro MUI_LANGUAGE "English"

;--------------------------------

; The stuff to install
Section "RobWork (required)"

  SectionIn RO
  
  ; Set output path to the installation directory.
  SetOutPath $INSTDIR
  
  CreateDirectory $INSTDIR\workspace
  CreateDirectory $INSTDIR\workspace\RobWork
  CreateDirectory $INSTDIR\workspace\RobWorkStudio
  
  File /r /x .svn workspace 
      
  ; Put file there
  ; File /r "workspace"
  File "RWPrompt.bat"
  !insertmacro ReplaceInFile "RWPrompt.bat" "ROBWORKPACKAGE" "$INSTDIR" 
  Delete "$INSTDIR\RWPrompt.bat.old"
  
  ExpandEnvStrings $0 "RW_DEPENDENCIES=$INSTDIR"
  
  ; Write the installation path into the registry
  WriteRegStr HKLM "Software\RobWorkVS2005Package" "Install_Dir" "$INSTDIR"
   
  ; Write the uninstall keys for Windows
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\RobWorkVS2005Package" "DisplayName" "RobWork MinGWPackage"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\RobWorkVS2005Package" "UninstallString" '"$INSTDIR\uninstall.exe"'
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\RobWorkVS2005Package" "NoModify" 1
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\RobWorkVS2005Package" "NoRepair" 1
  WriteUninstaller "uninstall.exe"
  
SectionEnd

Section "XercesC 2.8.0"

  SectionIn RO
  
  ; Set output path to the installation directory.
  SetOutPath $INSTDIR
  
  File /r xerces-c_2_8_0-x86-windows-vc_8_0  
SectionEnd

Section "Boost 1.39.0"

  SectionIn RO
  
  ; Set output path to the installation directory.
  SetOutPath $INSTDIR
  
  File /r boost_1_39_0
SectionEnd


; Optional section (can be disabled by the user)
Section "Start Menu Shortcuts"

  CreateDirectory "$SMPROGRAMS\RobWork"
  CreateShortCut "$SMPROGRAMS\RobWork\Uninstall.lnk" "$INSTDIR\uninstall.exe" "" "$INSTDIR\uninstall.exe" 0
  CreateShortCut "$SMPROGRAMS\RobWork\RobWork.lnk" "$INSTDIR\workspace\RobWork\projects\VisualStudio2005\RobWork.sln" "" "$INSTDIR\\workspace\RobWork\projects\VisualStudio2005\RobWork.sln" 0
  CreateShortCut "$SMPROGRAMS\RobWork\RobWork.lnk" "$INSTDIR\workspace\RobWorkStudio\projects\VisualStudio2005\RobWorkStudio.sln" "" "$INSTDIR\workspace\RobWorkStudio\projects\VisualStudio2005\RobWorkStudio.sln" 0

  
SectionEnd

;--------------------------------

; Uninstaller

Section "Uninstall"
  
  ; Remove registry keys
  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\RobWorkVS2005Package"
  DeleteRegKey HKLM SOFTWARE\RobWorkVS2005Package

  ; Remove files and uninstaller
  Delete $INSTDIR\RWPrompt.bat
  RMDir /r $INSTDIR\workspace 
  RMDir /r $INSTDIR\boost_1_39_0
  RMDir /r $INSTDIR\xerces-c_2_8_0-x86-windows-vc_8_0
  Delete $INSTDIR\uninstall.exe

  ; Remove shortcuts, if any
  Delete "$SMPROGRAMS\RobWork\*.*"

  ; Remove directories used
  RMDir "$SMPROGRAMS\RobWork"
  RMDir "$INSTDIR"

SectionEnd

Function RIF
 
  ClearErrors  ; want to be a newborn
 
  Exch $0      ; REPLACEMENT
  Exch
  Exch $1      ; SEARCH_TEXT
  Exch 2
  Exch $2      ; SOURCE_FILE
 
  Push $R0     ; SOURCE_FILE file handle
  Push $R1     ; temporary file handle
  Push $R2     ; unique temporary file name
  Push $R3     ; a line to sar/save
  Push $R4     ; shift puffer
 
  IfFileExists $2 +1 RIF_error      ; knock-knock
  FileOpen $R0 $2 "r"               ; open the door
 
  GetTempFileName $R2               ; who's new?
  FileOpen $R1 $R2 "w"              ; the escape, please!
 
  RIF_loop:                         ; round'n'round we go
    FileRead $R0 $R3                ; read one line
    IfErrors RIF_leaveloop          ; enough is enough
    RIF_sar:                        ; sar - search and replace
      Push "$R3"                    ; (hair)stack
      Push "$1"                     ; needle
      Push "$0"                     ; blood
      Call StrReplace               ; do the bartwalk
      StrCpy $R4 "$R3"              ; remember previous state
      Pop $R3                       ; gimme s.th. back in return!
      StrCmp "$R3" "$R4" +1 RIF_sar ; loop, might change again!
    FileWrite $R1 "$R3"             ; save the newbie
  Goto RIF_loop                     ; gimme more
 
  RIF_leaveloop:                    ; over'n'out, Sir!
    FileClose $R1                   ; S'rry, Ma'am - clos'n now
    FileClose $R0                   ; me 2
 
    Delete "$2.old"                 ; go away, Sire
    Rename "$2" "$2.old"            ; step aside, Ma'am
    Rename "$R2" "$2"               ; hi, baby!
 
    ClearErrors                     ; now i AM a newborn
    Goto RIF_out                    ; out'n'away
 
  RIF_error:                        ; ups - s.th. went wrong...
    SetErrors                       ; ...so cry, boy!
 
  RIF_out:                          ; your wardrobe?
  Pop $R4
  Pop $R3
  Pop $R2
  Pop $R1
  Pop $R0
  Pop $2
  Pop $0
  Pop $1
 
FunctionEnd

; StrReplace
; Replaces all ocurrences of a given needle within a haystack with another string
; Written by dandaman32
 
Var STR_REPLACE_VAR_0
Var STR_REPLACE_VAR_1
Var STR_REPLACE_VAR_2
Var STR_REPLACE_VAR_3
Var STR_REPLACE_VAR_4
Var STR_REPLACE_VAR_5
Var STR_REPLACE_VAR_6
Var STR_REPLACE_VAR_7
Var STR_REPLACE_VAR_8
 
Function StrReplace
  Exch $STR_REPLACE_VAR_2
  Exch 1
  Exch $STR_REPLACE_VAR_1
  Exch 2
  Exch $STR_REPLACE_VAR_0
    StrCpy $STR_REPLACE_VAR_3 -1
    StrLen $STR_REPLACE_VAR_4 $STR_REPLACE_VAR_1
    StrLen $STR_REPLACE_VAR_6 $STR_REPLACE_VAR_0
    loop:
      IntOp $STR_REPLACE_VAR_3 $STR_REPLACE_VAR_3 + 1
      StrCpy $STR_REPLACE_VAR_5 $STR_REPLACE_VAR_0 $STR_REPLACE_VAR_4 $STR_REPLACE_VAR_3
      StrCmp $STR_REPLACE_VAR_5 $STR_REPLACE_VAR_1 found
      StrCmp $STR_REPLACE_VAR_3 $STR_REPLACE_VAR_6 done
      Goto loop
    found:
      StrCpy $STR_REPLACE_VAR_5 $STR_REPLACE_VAR_0 $STR_REPLACE_VAR_3
      IntOp $STR_REPLACE_VAR_8 $STR_REPLACE_VAR_3 + $STR_REPLACE_VAR_4
      StrCpy $STR_REPLACE_VAR_7 $STR_REPLACE_VAR_0 "" $STR_REPLACE_VAR_8
      StrCpy $STR_REPLACE_VAR_0 $STR_REPLACE_VAR_5$STR_REPLACE_VAR_2$STR_REPLACE_VAR_7
      StrLen $STR_REPLACE_VAR_6 $STR_REPLACE_VAR_0
      Goto loop
    done:
  Pop $STR_REPLACE_VAR_1 ; Prevent "invalid opcode" errors and keep the
  Pop $STR_REPLACE_VAR_1 ; stack as it was before the function was called
  Exch $STR_REPLACE_VAR_0
FunctionEnd
 
