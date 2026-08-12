빌드·실행 방법                                                                                                           
===
cd cpp && cmake -S . -B build && cmake --build build                                                                     
cd build && ctest --output-on-failure

# Cmake
include(CTest)          # 여기서 BUILD_TESTING 옵션을 정의하고 기본값을 ON                                               
으로 설정                                                                                                                
if(BUILD_TESTING)       # → 기본 상태에서 항상 참

     include(CTest)는 내부적으로 option(BUILD_TESTING "..." ON)을 실행합니다.                                                 
     그래서 configure 시점에 값을 따로 안 주면 ON → if 통과 → test_libs 실행                                                  
     파일이 CMake 대상에 등록됩니다.                                                                                          
                                                                                                                              
     그리고 cmake --build build는 모든 대상을 빌드하는 명령이라 (수동으로 특정                                                
     타깃만 지정하지 않는 한) c_libs와 test_libs 둘 다 빌드됩니다. 테스트만                                                   
     실행되는 것도 아니라, ctest는 명시적으로 실행해야 돌아갑니다.                                                            
                                                                                                                              
     테스트를 꺼서 빌드하려면:                                                                                                
                                                                                        ⬖ Getting started                ✕    
     cmake -S . -B build -DBUILD_TESTING=OFF                                                                                  
     cmake --build build   # test_libs는 빌드 안 됨                                       OpenCode includes free models       
                                                                                          so you can start immediately.       
     반대로 지금처럼 기본 ON 상태가 "라이브러리 + 테스트 항상 같이 빌드"라서                                                  
     개발할 때 편합니다.                                              

---

1. CMake 구성 — cmake -S . -B build

    - -S . : 소스가 있는 디렉토리. 현재(cpp/)를 지정합니다.
    - -B build : 빌드 출력을 build/ 폴더에 만들라는 뜻.
    - 결과적으로 CMakeLists.txt를 읽고 환경(컴파일러, gtest 위치 등)을 탐색해                                                
      Makefile, 캐시 등을 build/에 생성합니다.

    2. 실제 컴파일 — cmake --build build

    - build/ 안의 Makefile을 호출해 libc_libs.a(정적 라이브러리)와 test_libs(                                                
      테스트 실행 파일)를 만들고 링크합니다.

    3. 테스트 실행 — cd build && ctest --output-on-failure

    - cd build : ctest는 build/ 안에서 실행해야 테스트 정의를 찾습니다.
    - ctest : CMAKE에서 등록된 테스트(gtest_discover_tests로 등록한 8개)를                                                   
      하나씩 실행.                                                                     ⬖ Getting started                ✕
    - --output-on-failure : 실패한 테스트가 있으면 그 상세 로그를 보여줍니다.                                                
      모든 게 통과하면 Passed만 표시.                                                    OpenCode includes free models       
      so you can start immediately.       
      흐름 요약 : 구성(설계도 생성) → 빌드(실제 컴파일) → 테스트(실행·검증).                                                   
      순서가 고정된 이유는 빌드 전 구성이 먼저 필요하기 때문입니다. 